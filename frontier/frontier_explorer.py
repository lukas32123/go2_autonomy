#!/usr/bin/env python3
"""
frontier_explorer.py — Stufe 3d + A: Autonomiezyklus komplett, mit Ziel-Timeout.

Bachelorarbeit Go2 / Isaac Lab — Welt-2-Autonomieknoten, Sprosse 5 (Kern der Arbeit).

Liest /map + Pose (TF), waehlt iterativ das beste Frontier-Ziel und schickt es an
Nav2 (navigate_to_pose). Nach Ankunft wird automatisch das naechste Ziel gewaehlt,
bis keine Frontiers mehr uebrig sind. Fehlgeschlagene Ziele -> Blacklist.

Zustandsautomat:
  IDLE -> NAVIGATING -> (Ankunft) -> IDLE -> ...
       -> (Frontiers erschoepft + Karte stabil) -> RETURNING -> (Home) -> DONE.
  - Gesendet wird nur beim Uebergang nach NAVIGATING; waehrend der Fahrt bleibt
    das Ziel fix (kein Zappeln).
  - 3c-Abschluss: RETURNING nur, wenn ueber 'completion_patience' aufeinanderfolgende
    Karten-Updates KEINE gueltige Frontier existiert UND die Karte in dieser Zeit
    STABIL ist (Zahl bekannter Zellen aendert sich um < 'map_stability_tol').
    Waechst/schwankt die Karte, wird der Abschlusszaehler zurueckgesetzt.
  - 3d-Return-to-Home: Startpose beim ERSTEN gueltigen TF-Fix gemerkt (map-Frame);
    nach Abschluss faehrt der Roboter dorthin zurueck (bestehende xy_goal_tolerance).
    Scheitert das Home-Ziel, bis MAX_HOME_ATTEMPTS neu versuchen, dann aufgeben.
  - A-Ziel-Timeout: Ist ein Explorations-Ziel laenger als 'goal_timeout' Sekunden
    aktiv, ohne erreicht zu werden, wird es gecancelt, geblacklistet und das
    naechste Ziel gewaehlt. Verhindert Endlos-Haenger an unerreichbaren Zielen
    (z. B. Frontier im soliden Block). Gilt NUR fuer NAVIGATING, nicht fuer
    RETURNING (das hat eigene Wiederhol-Logik).
  - Beim Beenden (Strg-C) wird ein laufendes Ziel aktiv gecancelt.

Marker: gruene Zellen, orange Zentroide, roter Pfeil (bester Kandidat),
goldene Saeule (aktives Ziel; in RETURNING = Home). Farben bewusst != Costmap.

Start (im Container welt2):
  python3 /root/frontier/frontier_explorer.py --ros-args -p use_sim_time:=true
"""

import sys
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import ColorRGBA
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

STATE_IDLE = "IDLE"
STATE_NAVIGATING = "NAVIGATING"
STATE_RETURNING = "RETURNING"
STATE_DONE = "DONE"

MAX_HOME_ATTEMPTS = 3   
MAX_GOAL_ATTEMPTS = 2   


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("marker_topic", "/frontiers")
        self.declare_parameter("min_cluster_size", 5)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("connectivity", 8)
        self.declare_parameter("max_occupied_neighbors", 0)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("size_weight", 0.05)
        self.declare_parameter("blacklist_radius", 0.5)      
        self.declare_parameter("completion_patience", 5)     
        self.declare_parameter("min_goal_distance", 0.5)     
                                                             
                                                             
        self.declare_parameter("goal_clearance", 0.0)
        self.declare_parameter("reachability_inflation", 0.3)

        self.declare_parameter("max_snap_candidates", 80)
        self.declare_parameter("map_stability_tol", 50)      
        self.declare_parameter("goal_timeout", 100.0)

        self.map_topic = self.get_parameter("map_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.min_cluster_size = int(self.get_parameter("min_cluster_size").value)
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.max_occ_nbr = int(self.get_parameter("max_occupied_neighbors").value)
        self.global_frame = self.get_parameter("global_frame").value
        self.robot_base_frame = self.get_parameter("robot_base_frame").value
        self.size_weight = float(self.get_parameter("size_weight").value)
        self.blacklist_radius = float(self.get_parameter("blacklist_radius").value)
        self.completion_patience = int(self.get_parameter("completion_patience").value)
        self.min_goal_distance = float(self.get_parameter("min_goal_distance").value)
        self.goal_clearance = float(self.get_parameter("goal_clearance").value)
        self.reach_infl = float(self.get_parameter("reachability_inflation").value)
        self.max_snap_candidates = int(self.get_parameter("max_snap_candidates").value)
        self.map_stability_tol = int(self.get_parameter("map_stability_tol").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout").value)
        conn = int(self.get_parameter("connectivity").value)
        if conn == 4:
            self.nbrs = ((1, 0), (-1, 0), (0, 1), (0, -1))
        else:
            self.nbrs = ((1, 0), (-1, 0), (0, 1), (0, -1),
                         (1, 1), (1, -1), (-1, 1), (-1, -1))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.state = STATE_IDLE
        self.blacklist = []              
        self.active_goal = None          
        self.current_goal_handle = None
        self.empty_count = 0
        self.goals_reached = 0
        self.shutting_down = False
        self.latest_valid = []           
        self.latest_robot = None         
        self.last_known_count = None     
        self.map_is_stable = False       

        self.home_pose = None           
        self.home_attempts = 0           

        self.goal_start_time = None      
        self.goal_timed_out = False      

        self.goal_invalidated = False    
        self.invalid_reason = ""         
        self._clear_offsets = None       
        self._clear_offsets_rad = -1
        # D: Fehlschlag-Zaehler je Zielort (10-cm-Raster)
        self.goal_failures = {}
        
        self.t_start = None              
        self.t_explore_end = None        
        self.t_home = None               
        self.dist_total = 0.0            
        self.dist_at_explore_end = None  
        self.last_odom_xy = None
        self.n_revalidated = 0           
        self.n_retries = 0               
        self.n_timeouts = 0              
        self.rth_ok = None               
        self.eval_printed = False

        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_cb, map_qos
        )
        # E: gefahrene Distanz aus /odom integrieren (Ground Truth, voller Takt).
        #    Ueber TF/Karten-Updates (0.5 Hz) waere die Strecke stark unterschaetzt.
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 20)
        self.pub = self.create_publisher(MarkerArray, self.marker_topic, 1)

        # A: 1-Hz-Timer prueft den Ziel-Timeout (nutzt Node-Clock -> sim time)
        self.timeout_timer = self.create_timer(1.0, self.check_goal_timeout)

        self.get_logger().info(
            f"[3d] Autonomiezyklus aktiv: sub '{self.map_topic}', pub '{self.marker_topic}'; "
            f"size_weight={self.size_weight}, blacklist_radius={self.blacklist_radius}, "
            f"min_goal_distance={self.min_goal_distance}, goal_clearance={self.goal_clearance}, "
            f"max_goal_attempts={MAX_GOAL_ATTEMPTS}, revalidierung=an, "
            f"completion_patience={self.completion_patience}, "
            f"map_stability_tol={self.map_stability_tol}, goal_timeout={self.goal_timeout}s, "
            f"frames {self.global_frame}->{self.robot_base_frame}"
        )

    # ------------------------------------------------------------------
    def map_cb(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        data = msg.data

        if w == 0 or h == 0 or len(data) != w * h:
            self.get_logger().warn(
                f"Karte uebersprungen (malformed): width={w} height={h} data={len(data)}",
                throttle_duration_sec=5.0,
            )
            return

        # --- 1) Frontier-Zellen (+ Zahl bekannter Zellen fuer 3c) ---
        frontier = bytearray(w * h)
        occ = self.occupied_threshold
        max_occ = self.max_occ_nbr
        nbrs = self.nbrs
        known = 0

        # --- BEST-PRACTICE: belegte Zellen um reach_infl verdicken -----------
        # blocked[i] = 1, wenn Zelle i belegt ODER innerhalb reach_infl um eine
        # belegte Zelle. Solche Zellen werden bei der Detektion wie Hindernis
        # behandelt -> Frontiers in zu engen Luecken entstehen nicht.
        # reach_infl <= 0 -> blocked=None -> exakt bisheriges Verhalten.
        infl_cells = int(math.ceil(self.reach_infl / res)) if self.reach_infl > 0.0 else 0
        if infl_cells > 0:
            blocked = bytearray(w * h)
            ring = []
            for i in range(w * h):
                if data[i] >= occ and data[i] >= 0:
                    blocked[i] = 1
                    ring.append(i)
            for _ in range(infl_cells):
                nxt = []
                for idx in ring:
                    r = idx // w
                    c = idx % w
                    for dc, dr in nbrs:
                        nc = c + dc
                        nr = r + dr
                        if 0 <= nc < w and 0 <= nr < h:
                            nidx = nr * w + nc
                            if not blocked[nidx]:
                                blocked[nidx] = 1
                                nxt.append(nidx)
                ring = nxt
        else:
            blocked = None
        # --------------------------------------------------------------------

        for row in range(h):
            base = row * w
            for col in range(w):
                v = data[base + col]
                if v < 0:
                    continue          # unbekannt -> nicht bekannt, keine Frontier
                known += 1             # frei oder belegt = bekannt (3c)
                if v >= occ:
                    continue          # belegt -> keine Frontier-Kandidatin
                if blocked is not None and blocked[base + col]:
                    continue          # BEST-PRACTICE: in Inflationszone -> keine Frontier
                has_unknown = False
                occ_count = 0
                for dc, dr in nbrs:
                    nc = col + dc
                    nr = row + dr
                    if 0 <= nc < w and 0 <= nr < h:
                        nv = data[nr * w + nc]
                        if nv < 0:
                            has_unknown = True
                        elif nv >= occ:
                            occ_count += 1
                if has_unknown and occ_count <= max_occ:
                    frontier[base + col] = 1

        n_cells = sum(frontier)

        # --- 3c) Karten-Stabilitaet: aendern sich bekannte Zellen kaum? ---
        if self.last_known_count is None:
            self.map_is_stable = False
        else:
            self.map_is_stable = abs(known - self.last_known_count) <= self.map_stability_tol
        self.last_known_count = known

        # --- 2) Clustering (BFS) ---
        clusters = []
        visited = bytearray(w * h)
        for start in range(w * h):
            if frontier[start] and not visited[start]:
                comp = []
                q = deque((start,))
                visited[start] = 1
                while q:
                    idx = q.popleft()
                    comp.append(idx)
                    r = idx // w
                    c = idx % w
                    for dc, dr in nbrs:
                        nc = c + dc
                        nr = r + dr
                        if 0 <= nc < w and 0 <= nr < h:
                            nidx = nr * w + nc
                            if frontier[nidx] and not visited[nidx]:
                                visited[nidx] = 1
                                q.append(nidx)
                clusters.append(comp)

        # --- 3) Filtern -> Zentrum + Snap ---
        valid = []
        skipped_clear = 0          # B: Cluster ohne einnehmbare Zielzelle (Wand-Artefakte)
        clear_r = int(math.ceil(self.goal_clearance / res)) if self.goal_clearance > 0.0 else 0
        for comp in clusters:
            n = len(comp)
            if n < self.min_cluster_size:
                continue
            sx = sy = 0.0
            cand = []
            for idx in comp:
                r = idx // w
                c = idx % w
                px = ox + (c + 0.5) * res
                py = oy + (r + 0.5) * res
                cand.append((px, py, c, r))
                sx += px
                sy += py
            cx = sx / n
            cy = sy / n
            # B: Snap = naechste Zelle zum Zentroid, die genug Wandabstand hat.
            #    Bei clear_r == 0 ist das exakt das alte Verhalten (naechste Zelle).
            cand.sort(key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
            goal = None
            for (px, py, c, r) in cand[: self.max_snap_candidates]:
                if self.has_clearance(data, w, h, c, r, clear_r, occ):
                    goal = (px, py)
                    break
            if goal is None:
                skipped_clear += 1   # keine Zelle im Cluster ist fuer den Footprint
                continue             # einnehmbar -> Wand-Artefakt, verwerfen
            valid.append((cx, cy, n, goal[0], goal[1]))

        # --- 4) Pose (TF) ---
        robot = None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_base_frame, Time()
            )
            robot = (tf.transform.translation.x, tf.transform.translation.y)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF {self.global_frame}->{self.robot_base_frame} nicht verfuegbar: {e}",
                throttle_duration_sec=5.0,
            )

        self.latest_valid = valid
        self.latest_robot = robot

        # --- 3d) Home-Pose einmalig beim ersten gueltigen TF-Fix merken ---
        # (passiert vor dem ersten Ziel -> tatsaechliche Startpose im map-Frame)
        if self.home_pose is None and robot is not None:
            self.home_pose = robot
            self.t_start = self.get_clock().now()      # E: Startzeit (Sim)
            self.dist_total = 0.0                      # E: Zaehler auf 0
            self.get_logger().info(
                f"[3d] Home-Pose gemerkt (Startpose): ({robot[0]:.2f}, {robot[1]:.2f})."
            )

        best = self.select_best(valid, robot)

        sizes = sorted((v[2] for v in valid), reverse=True)
        self.get_logger().info(
            f"[{self.state}] Frontiers: {n_cells} Zellen | {len(valid)} gueltig | "
            f"{skipped_clear} verworfen (Clearance) | "
            f"Groessen={sizes[:5]} | Karte={'stabil' if self.map_is_stable else 'waechst'} | "
            f"Blacklist={len(self.blacklist)} | Ziele erreicht={self.goals_reached}"
        )

        self.publish_markers(msg.header, frontier, w, h, res, ox, oy, valid, best)

        # --- 5) C: aktives Ziel gegen die AKTUELLE Karte pruefen ---
        if self.state == STATE_NAVIGATING and not self.goal_invalidated:
            if not self.goal_still_valid(data, w, h, res, ox, oy, occ, clear_r):
                self.abort_goal_invalid()
                return

        # --- 6) Zyklus: nur im IDLE ein neues Ziel waehlen/senden ---
        if self.state == STATE_IDLE:
            self.try_select_and_send()

    # ------------------------------------------------------------------
    # E: Distanzintegration (Expose 5.3 "zurueckgelegte Distanz")
    def odom_cb(self, msg):
        p = msg.pose.pose.position
        xy = (p.x, p.y)
        if self.last_odom_xy is not None:
            dx = xy[0] - self.last_odom_xy[0]
            dy = xy[1] - self.last_odom_xy[1]
            step = math.hypot(dx, dy)
            if step < 0.5:               # > 0.5 m in einem Takt = Glitch, ignorieren
                self.dist_total += step
        self.last_odom_xy = xy

    # ------------------------------------------------------------------
    def is_blacklisted(self, gx, gy):
        r2 = self.blacklist_radius ** 2
        for (bx, by) in self.blacklist:
            if (gx - bx) ** 2 + (gy - by) ** 2 <= r2:
                return True
        return False

    def clearance_offsets(self, rad):
        """C: Kreisscheiben-Offsets, nach Abstand aufsteigend, gecacht.

        Aufsteigend, weil eine Wand meist NAH ist, wenn sie ueberhaupt stoert --
        so bricht has_clearance im Ablehnungsfall so frueh wie moeglich ab.
        """
        if rad != self._clear_offsets_rad:
            r2 = rad * rad
            offs = [(dc, dr)
                    for dr in range(-rad, rad + 1)
                    for dc in range(-rad, rad + 1)
                    if dc * dc + dr * dr <= r2]
            offs.sort(key=lambda o: o[0] * o[0] + o[1] * o[1])
            self._clear_offsets = tuple(offs)
            self._clear_offsets_rad = rad
        return self._clear_offsets

    def has_clearance(self, data, w, h, c0, r0, rad, occ):
        """B/C: True, wenn im Zellradius 'rad' um (c0, r0) keine BELEGTE Zelle liegt.

        Warum 0.70 m (= inflation_radius aus nav2_go2.yaml): Der InflationLayer setzt
        JENSEITS dieses Radius Kosten 0. Innerhalb steigen sie exponentiell (bei 0.35 m
        noch ~140). MPPI wird dort vom CostCritic von der Wand weggedrueckt, waehrend der
        GoalCritic hinzieht -- Ergebnis sind Befehle wie vx=0.06 m/s. Gemessen: unterhalb
        von ~0.08 m/s setzt die gelernte Policy KEINEN Fuss (Totbereich). Der Roboter
        steht, Nav2 meldet "Failed to make progress", der BT retryt endlos.
        Also: solche Ziele gar nicht erst erzeugen.

        UNBEKANNTE Zellen (< 0) zaehlen NICHT als Hindernis -- sonst haette per
        Definition keine einzige Frontier-Zelle jemals Clearance. Genau daraus folgt
        aber die Restluecke, die goal_still_valid() schliesst: gegen eine Wand, die
        beim Zielwahl-Zeitpunkt noch UNBEKANNT ist, kann dieser Filter nichts tun.
        """
        if rad <= 0:
            return True                      # Filter aus -> altes Verhalten
        for (dc, dr) in self.clearance_offsets(rad):
            rr = r0 + dr
            if rr < 0 or rr >= h:
                continue                     # Kartenrand ist kein Hindernis
            cc = c0 + dc
            if cc < 0 or cc >= w:
                continue
            if data[rr * w + cc] >= occ:
                return False
        return True

    # ------------------------------------------------------------------
    # C: Laufende Revalidierung des AKTIVEN Ziels
    def goal_still_valid(self, data, w, h, res, ox, oy, occ, clear_r):
        """C: Ist das aktive Ziel auf der AKTUELLEN Karte noch anfahrbar?

        Ein Frontier-Ziel liegt per Definition an der Grenze zum Unbekannten. Waehrend
        der Anfahrt kann dort eine Wand auftauchen -- real beobachtet: Ziel (-3.03, -0.89)
        lag 3 cm vor der Innenblock-Wand (x = -3.00), die zum Wahlzeitpunkt unbekannt war.
        Nav2 haelt stur am Ziel fest (Restpfad sprang von 2 m auf 9.9 m), MPPI steckt im
        Kostenberg fest, der Roboter steht 80 s. Deshalb: bei jedem Karten-Update pruefen.
        """
        if self.active_goal is None:
            return True
        gx, gy = self.active_goal
        c = int((gx - ox) / res)
        r = int((gy - oy) / res)
        if c < 0 or c >= w or r < 0 or r >= h:
            return True          # ausserhalb der Karte -> nicht beurteilbar
        if data[r * w + c] >= occ:
            self.invalid_reason = "Zielzelle ist inzwischen BELEGT"
            return False
        if not self.has_clearance(data, w, h, c, r, clear_r, occ):
            self.invalid_reason = (
                f"Wandabstand unter goal_clearance ({self.goal_clearance:.2f} m) gefallen"
            )
            return False
        return True

    def abort_goal_invalid(self):
        """C: Aktives Ziel abbrechen -- OHNE Blacklist.

        Bewusst keine Blacklist: Der Cluster als solcher bleibt gueltig. Die Wand ist
        jetzt BEKANNT, also verwirft der Snap beim naechsten Update die wandnahen Zellen
        von selbst und waehlt eine mit Abstand. Blacklisten wuerde die Region unnoetig
        aufgeben -- genau der Fehler, der in Lauf B eine Kartenluecke hinterliess.
        """
        self.get_logger().warn(
            f"[C] Ziel {self.active_goal} ist ungueltig geworden: {self.invalid_reason} "
            f"-> abbrechen, Neuwahl (KEINE Blacklist)."
        )
        self.goal_invalidated = True
        self.n_revalidated += 1                        # E
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
        else:
            self.state = STATE_IDLE

    def goal_key(self, goal):
        """D: Zielort auf 10-cm-Raster -- 'derselbe Ort' trotz Snap-Jitter."""
        if goal is None:
            return None
        return (round(goal[0], 1), round(goal[1], 1))

    def select_best(self, valid, robot):
        """Bestes nicht-blacklistetes Cluster (min Kosten), Ziel mind. min_goal_distance entfernt."""
        if robot is None or not valid:
            return None
        rx, ry = robot
        best = None
        best_cost = None
        for (cx, cy, n, gx, gy) in valid:
            if self.is_blacklisted(gx, gy):
                continue
            dist = math.hypot(gx - rx, gy - ry)
            if dist < self.min_goal_distance:
                continue
            cost = dist - self.size_weight * n
            if best_cost is None or cost < best_cost:
                best_cost = cost
                yaw = math.atan2(gy - ry, gx - rx)
                best = (gx, gy, n, dist, cost, yaw)
        return best

    def try_select_and_send(self):
        if self.shutting_down or self.state != STATE_IDLE:
            return
        best = self.select_best(self.latest_valid, self.latest_robot)
        if best is None:
            if self.latest_robot is None:
                return  # Pose fehlt -> nicht als "fertig" werten
            # 3c: nur auf STABILER Karte in Richtung Abschluss zaehlen.
            if not self.map_is_stable:
                if self.empty_count > 0:
                    self.get_logger().info(
                        "[3c] Karte noch instabil/waechst -> Abschlusszaehler zurueckgesetzt."
                    )
                self.empty_count = 0
                return
            self.empty_count += 1
            self.get_logger().info(
                f"[3c] stabile Karte, keine gueltigen Frontiers "
                f"({self.empty_count}/{self.completion_patience})."
            )
            if self.empty_count >= self.completion_patience:
                self.get_logger().info(
                    f"[3c] ===> EXPLORATION FERTIG: keine Frontiers mehr, Karte stabil ueber "
                    f"{self.completion_patience} Updates. {self.goals_reached} Ziele erreicht. "
                    f"Starte Return-to-Home."
                )
                # E: Explorationsphase abgeschlossen -- Zeit und Distanz einfrieren
                self.t_explore_end = self.get_clock().now()
                self.dist_at_explore_end = self.dist_total
                self.start_return_home()
            return
        self.empty_count = 0
        self.send_goal(best)

    def send_goal(self, best):
        gx, gy, n, dist, cost, yaw = best
        goal_msg = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self.global_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = gx
        ps.pose.position.y = gy
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        goal_msg.pose = ps

        self.state = STATE_NAVIGATING
        self.active_goal = (gx, gy)
        # A: Startzeit des Ziels merken, Timeout-Flag zuruecksetzen
        self.goal_start_time = self.get_clock().now()
        self.goal_timed_out = False
        self.get_logger().info(
            f"[3b] Ziel #{self.goals_reached + 1}: ({gx:.2f}, {gy:.2f}), "
            f"Cluster {n} Zellen, Dist {dist:.2f} m -> NAVIGATING"
        )
        fut = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        fut.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn(f"[3b] Ziel ABGELEHNT -> Blacklist {self.active_goal}.")
            if self.active_goal is not None:
                self.blacklist.append(self.active_goal)
            self.current_goal_handle = None
            self.state = STATE_IDLE
            return
        self.current_goal_handle = gh
        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self.result_cb)

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"[{self.state}] unterwegs... Restdistanz {fb.distance_remaining:.2f} m",
            throttle_duration_sec=3.0,
        )

    def result_cb(self, future):
        status = future.result().status
        self.current_goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.goals_reached += 1
            self.get_logger().info(
                f"[3b] ===> ERREICHT (#{self.goals_reached}) -> naechstes Ziel."
            )
            self.state = STATE_IDLE
        elif status == GoalStatus.STATUS_CANCELED:
            # C: Cancel durch Ziel-Invalidierung -> KEINE Blacklist, sofort neu waehlen
            if self.goal_invalidated:
                self.goal_invalidated = False
                self.get_logger().info("[C] Ziel abgebrochen (Karte hat es entwertet). Neuwahl.")
                self.state = STATE_IDLE
                return
            # A: Cancel durch Timeout vs. Cancel durch Shutdown unterscheiden
            if self.goal_timed_out:
                self.goal_timed_out = False
                if self.active_goal is not None:
                    self.blacklist.append(self.active_goal)
                self.get_logger().warn(
                    f"[timeout] Ziel verworfen -> Blacklist ({len(self.blacklist)}). Naechstes Ziel."
                )
                self.state = STATE_IDLE
            else:
                self.get_logger().warn("[3b] Ziel canceled (Shutdown).")
                self.state = STATE_IDLE
        else:
            # D: NICHT beim ersten Fehlschlag blacklisten.
            #   Real beobachtet: Der Explorer liest /map direkt und feuert ein Ziel im
            #   frisch dazugekommenen Randstreifen; Nav2s global_costmap verdaut dieselbe
            #   Kartenerweiterung ~8 ms spaeter und lehnt mit "outside bounds" ab. Das
            #   groesste Cluster des Laufs (1283 Zellen) ging so an einem Millisekunden-
            #   Rennen verloren. Ein einziger Retry loest das.
            key = self.goal_key(self.active_goal)
            self.goal_failures[key] = self.goal_failures.get(key, 0) + 1
            fails = self.goal_failures[key]
            if fails < MAX_GOAL_ATTEMPTS:
                self.n_retries += 1                    # E
                self.get_logger().warn(
                    f"[D] FEHLGESCHLAGEN (Status {status}) bei {self.active_goal} "
                    f"-- Versuch {fails}/{MAX_GOAL_ATTEMPTS}, KEINE Blacklist, Neuwahl."
                )
            else:
                if self.active_goal is not None:
                    self.blacklist.append(self.active_goal)
                self.get_logger().warn(
                    f"[3b] FEHLGESCHLAGEN (Status {status}) nach {fails} Versuchen "
                    f"-> Blacklist {self.active_goal} ({len(self.blacklist)})."
                )
            self.state = STATE_IDLE

    # ------------------------------------------------------------------
    # A: Ziel-Timeout (nur fuer Explorations-Ziele, Zustand NAVIGATING)
    def check_goal_timeout(self):
        if self.shutting_down or self.state != STATE_NAVIGATING:
            return
        if self.goal_timed_out or self.goal_invalidated or self.goal_start_time is None:
            return  # bereits ausgeloest / wird gerade invalidiert / kein aktives Ziel
        elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
        if elapsed >= self.goal_timeout:
            self.get_logger().warn(
                f"[timeout] Ziel {self.active_goal} seit {elapsed:.0f}s aktiv "
                f"(> {self.goal_timeout:.0f}s) -> cancele."
            )
            self.goal_timed_out = True   # Blacklist erfolgt im Result-Callback (CANCELED)
            self.n_timeouts += 1         # E
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()

    # ------------------------------------------------------------------
    # 3d: Return-to-Home
    def start_return_home(self):
        if self.home_pose is None:
            self.get_logger().warn(
                "[3d] Keine Home-Pose gemerkt -> kann nicht zurueckkehren. -> DONE."
            )
            self.state = STATE_DONE
            self.active_goal = None
            return
        hx, hy = self.home_pose
        self.state = STATE_RETURNING
        self.active_goal = (hx, hy)
        goal_msg = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self.global_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = hx
        ps.pose.position.y = hy
        ps.pose.orientation.w = 1.0   # Ziel-Ausrichtung egal (yaw_goal_tolerance weit offen)
        goal_msg.pose = ps
        self.get_logger().info(
            f"[3d] ===> RETURN-TO-HOME (Versuch {self.home_attempts + 1}/{MAX_HOME_ATTEMPTS}): "
            f"fahre zur Startpose ({hx:.2f}, {hy:.2f})."
        )
        fut = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        fut.add_done_callback(self.home_response_cb)

    def home_response_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.current_goal_handle = None
            self.home_attempts += 1
            if self.home_attempts < MAX_HOME_ATTEMPTS:
                self.get_logger().warn(
                    f"[3d] Home-Ziel ABGELEHNT ({self.home_attempts}/{MAX_HOME_ATTEMPTS}) "
                    f"-> neuer Versuch."
                )
                self.start_return_home()
            else:
                self.get_logger().warn(
                    f"[3d] Home-Ziel nach {MAX_HOME_ATTEMPTS} Versuchen abgelehnt -> DONE (aufgegeben)."
                )
                self.state = STATE_DONE
                self.active_goal = None
            return
        self.current_goal_handle = gh
        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self.home_result_cb)

    def home_result_cb(self, future):
        status = future.result().status
        self.current_goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.state = STATE_DONE
            self.active_goal = None
            self.t_home = self.get_clock().now()       # E
            self.rth_ok = True                         # E
            self.get_logger().info(
                f"[3d] ===> HOME ERREICHT: zurueck an der Startpose. "
                f"{self.goals_reached} Frontier-Ziele erkundet. Autonomiezyklus komplett."
            )
            self.print_eval()                          # E
            return
        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("[3d] Home-Ziel canceled (Shutdown).")
            self.state = STATE_DONE
            self.active_goal = None
            return
        # fehlgeschlagen
        self.home_attempts += 1
        if self.home_attempts < MAX_HOME_ATTEMPTS:
            self.get_logger().warn(
                f"[3d] Home-Ziel FEHLGESCHLAGEN (Status {status}, "
                f"{self.home_attempts}/{MAX_HOME_ATTEMPTS}) -> neuer Versuch."
            )
            self.start_return_home()
        else:
            self.get_logger().warn(
                f"[3d] Home nach {MAX_HOME_ATTEMPTS} Versuchen nicht erreicht -> DONE (aufgegeben)."
            )
            self.state = STATE_DONE
            self.active_goal = None
            self.t_home = self.get_clock().now()       # E
            self.rth_ok = False                        # E
            self.print_eval()                          # E

    # ------------------------------------------------------------------
    # E: Auswertung nach Expose 5.3 -- eine Zeile pro Lauf, direkt uebernehmbar
    def print_eval(self):
        if self.eval_printed:
            return
        self.eval_printed = True

        def secs(a, b):
            if a is None or b is None:
                return None
            return (b - a).nanoseconds / 1e9

        t_expl = secs(self.t_start, self.t_explore_end)
        t_rth = secs(self.t_explore_end, self.t_home)
        t_ges = secs(self.t_start, self.t_home)
        d_expl = self.dist_at_explore_end
        d_rth = (self.dist_total - d_expl) if d_expl is not None else None
        d_ges = self.dist_total

        def f(v, nk=1):
            return "n/a" if v is None else f"{v:.{nk}f}"

        rth = {True: "ERFOLGREICH", False: "FEHLGESCHLAGEN"}.get(self.rth_ok, "ABGEBROCHEN")

        L = self.get_logger()
        L.info("=" * 64)
        L.info("[EVAL] LAUF-METRIKEN (Expose 5.3) -- ALLE ZEITEN IN SIM-ZEIT")
        L.info("=" * 64)
        L.info(f"[EVAL] Ziele erreicht         : {self.goals_reached}")
        L.info(f"[EVAL] Explorationszeit       : {f(t_expl)} s")
        L.info(f"[EVAL] Return-to-Home-Zeit    : {f(t_rth)} s")
        L.info(f"[EVAL] Gesamtzeit             : {f(t_ges)} s")
        L.info(f"[EVAL] Distanz Exploration    : {f(d_expl, 2)} m")
        L.info(f"[EVAL] Distanz Return-to-Home : {f(d_rth, 2)} m")
        L.info(f"[EVAL] Distanz gesamt         : {f(d_ges, 2)} m")
        L.info(f"[EVAL] Return-to-Home         : {rth}")
        L.info(f"[EVAL] Blacklist-Eintraege    : {len(self.blacklist)}")
        L.info(f"[EVAL] Ziel-Revalidierungen(C): {self.n_revalidated}")
        L.info(f"[EVAL] Ziel-Retries (D)       : {self.n_retries}")
        L.info(f"[EVAL] Ziel-Timeouts (A)      : {self.n_timeouts}")
        L.info(f"[EVAL] Konfig                 : goal_clearance={self.goal_clearance} "
               f"min_goal_distance={self.min_goal_distance} size_weight={self.size_weight}")
        # Eine Zeile fuer die Tabelle. Semikolon = deutsches Excel.
        L.info(
            "[EVAL] CSV;ziele;t_expl;t_rth;t_ges;d_expl;d_rth;d_ges;rth;blacklist;reval;retry;timeout;clearance"
        )
        L.info(
            f"[EVAL] CSV;{self.goals_reached};{f(t_expl)};{f(t_rth)};{f(t_ges)};"
            f"{f(d_expl, 2)};{f(d_rth, 2)};{f(d_ges, 2)};{rth};{len(self.blacklist)};"
            f"{self.n_revalidated};{self.n_retries};{self.n_timeouts};{self.goal_clearance}"
        )
        L.info("=" * 64)
        L.info("[EVAL] HINWEIS: Karte JETZT sichern, solange SLAM laeuft:")
        L.info("[EVAL]   ros2 run nav2_map_server map_saver_cli -f /root/maps/<name> -t /map")
        L.info("=" * 64)

    def cancel_active_goal(self):
        self.shutting_down = True
        self.print_eval()          # E: auch bei Strg-C die Teilmetriken ausgeben
        if self.current_goal_handle is not None:
            self.get_logger().info("[3b] Beende -> cancele laufendes Nav2-Ziel...")
            try:
                fut = self.current_goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
            except Exception as e:
                self.get_logger().warn(f"[3b] Cancel fehlgeschlagen: {e}")

    # ------------------------------------------------------------------
    def publish_markers(self, header, frontier, w, h, res, ox, oy, valid, best):
        arr = MarkerArray()

        cells = Marker()
        cells.header = header
        cells.ns = "frontier_cells"
        cells.id = 0
        cells.type = Marker.CUBE_LIST
        cells.action = Marker.ADD
        cells.scale.x = res
        cells.scale.y = res
        cells.scale.z = max(res * 0.2, 0.01)
        cells.color = ColorRGBA(r=0.1, g=1.0, b=0.2, a=0.9)  # gruen
        cells.pose.orientation.w = 1.0
        for idx in range(w * h):
            if frontier[idx]:
                r = idx // w
                c = idx % w
                cells.points.append(Point(x=ox + (c + 0.5) * res,
                                          y=oy + (r + 0.5) * res, z=0.06))
        arr.markers.append(cells)

        cent = Marker()
        cent.header = header
        cent.ns = "frontier_centroids"
        cent.id = 1
        cent.type = Marker.SPHERE_LIST
        cent.action = Marker.ADD
        cent.scale.x = 0.15
        cent.scale.y = 0.15
        cent.scale.z = 0.15
        cent.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=1.0)  # orange
        cent.pose.orientation.w = 1.0
        for cx, cy, n, gx, gy in valid:
            cent.points.append(Point(x=cx, y=cy, z=0.15))
        arr.markers.append(cent)

        goal = Marker()
        goal.header = header
        goal.ns = "frontier_goal"
        goal.id = 2
        goal.type = Marker.ARROW
        if best is not None:
            gx, gy, n, dist, cost, yaw = best
            goal.action = Marker.ADD
            goal.scale.x = 0.5
            goal.scale.y = 0.1
            goal.scale.z = 0.1
            goal.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # rot
            goal.pose.position.x = gx
            goal.pose.position.y = gy
            goal.pose.position.z = 0.2
            goal.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            goal.action = Marker.DELETE
        arr.markers.append(goal)

        act = Marker()
        act.header = header
        act.ns = "active_goal"
        act.id = 3
        act.type = Marker.CYLINDER
        if self.active_goal is not None:
            ax, ay = self.active_goal
            act.action = Marker.ADD
            act.scale.x = 0.2
            act.scale.y = 0.2
            act.scale.z = 0.8
            act.color = ColorRGBA(r=1.0, g=0.85, b=0.0, a=0.85)  # gold
            act.pose.position.x = ax
            act.pose.position.y = ay
            act.pose.position.z = 0.4
            act.pose.orientation.w = 1.0
        else:
            act.action = Marker.DELETE
        arr.markers.append(act)

        self.pub.publish(arr)


def main():
    rclpy.init(args=sys.argv)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cancel_active_goal()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

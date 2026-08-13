#!/usr/bin/env python3
"""
Autonomer Frontier-Explorationsknoten fuer den Unitree Go2.

Der Knoten liest das Belegungsgitter, waehlt das guenstigste Grenzzellenziel und laesst es von Nav2 anfahren. Nach
jeder Ankunft folgt das naechste Ziel, bis keine gueltige Grenze mehr existiert
und die Karte stabil ist. Danach kehrt der Roboter zur gemerkten Startpose
zurueck.

Der Ablauf ist als Zustandsautomat organisiert.

    IDLE -> NAVIGATING -> (Ankunft) -> IDLE -> ...
         -> (Grenzen erschoepft und Karte stabil) -> RETURNING -> DONE

Ein Ziel wird nur beim Uebergang nach NAVIGATING gesendet. Waehrend der Fahrt
bleibt es fest, damit der Roboter nicht zwischen Zielen pendelt.

Vier Mechanismen sichern die Robustheit gegen unerreichbare oder entwertete
Ziele. Die laufende Revalidierung bricht ein Ziel ab, dessen Zelle inzwischen
belegt ist. Der einmalige Wiederholversuch faengt das Wettrennen zwischen der
direkt gelesenen Karte und der etwas spaeter aktualisierten Kostenkarte von
Nav2 ab. Der Ziel-Timeout verwirft ein Ziel, das zu lange aktiv bleibt, ohne
erreicht zu werden. Die Abschlusspruefung verlangt vor der Rueckkehr eine ueber
mehrere Aktualisierungen stabile Karte ohne gueltige Grenze.

Verantwortlichkeiten sind auf eigene Klassen verteilt:

``FrontierDetector`` gewinnt aus einer Karte die gueltigen Ziele. 
``GoalSelector`` waehlt daraus das beste aus. 
``Blacklist`` verwaltet gesperrte Orte und Wiederholzaehler.
``RunMetrics`` fuehrt Zeiten, Distanz und Zaehler und gibt die Auswertung aus.
``MarkerPublisher`` erzeugt die Anzeige fuer RViz. 
``FrontierExplorerNode`` haelt den Zustandsautomaten und die Anbindung an ROS zusammen.

Start im Container:
    python3 frontier_explorer.py --ros-args -p use_sim_time:=true
"""

import sys
import math
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

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


# ======================================================================
# Konfiguration
# ======================================================================
@dataclass
class ExplorerConfig:
    """Sammelt die ueber ROS-Parameter einstellbaren Groessen an einem Ort."""

    map_topic: str
    marker_topic: str
    min_cluster_size: int
    occupied_threshold: int
    max_occ_nbr: int
    global_frame: str
    robot_base_frame: str
    size_weight: float
    blacklist_radius: float
    completion_patience: int
    min_goal_distance: float
    goal_clearance: float
    reach_infl: float
    max_snap_candidates: int
    map_stability_tol: int
    goal_timeout: float
    nbrs: Tuple[Tuple[int, int], ...]

    @classmethod
    def from_node(cls, node: Node) -> "ExplorerConfig":
        """Deklariert die Parameter am Knoten und liest sie in eine Konfiguration."""
        node.declare_parameter("map_topic", "/map")
        node.declare_parameter("marker_topic", "/frontiers")
        node.declare_parameter("min_cluster_size", 5)
        node.declare_parameter("occupied_threshold", 65)
        node.declare_parameter("connectivity", 8)
        node.declare_parameter("max_occupied_neighbors", 0)
        node.declare_parameter("global_frame", "map")
        node.declare_parameter("robot_base_frame", "base_link")
        node.declare_parameter("size_weight", 0.05)
        node.declare_parameter("blacklist_radius", 0.5)
        node.declare_parameter("completion_patience", 5)
        node.declare_parameter("min_goal_distance", 0.5)
        node.declare_parameter("goal_clearance", 0.0)
        node.declare_parameter("reachability_inflation", 0.3)
        node.declare_parameter("max_snap_candidates", 80)
        node.declare_parameter("map_stability_tol", 50)
        node.declare_parameter("goal_timeout", 100.0)

        conn = int(node.get_parameter("connectivity").value)
        if conn == 4:
            nbrs = ((1, 0), (-1, 0), (0, 1), (0, -1))
        else:
            nbrs = ((1, 0), (-1, 0), (0, 1), (0, -1),
                    (1, 1), (1, -1), (-1, 1), (-1, -1))

        return cls(
            map_topic=node.get_parameter("map_topic").value,
            marker_topic=node.get_parameter("marker_topic").value,
            min_cluster_size=int(node.get_parameter("min_cluster_size").value),
            occupied_threshold=int(node.get_parameter("occupied_threshold").value),
            max_occ_nbr=int(node.get_parameter("max_occupied_neighbors").value),
            global_frame=node.get_parameter("global_frame").value,
            robot_base_frame=node.get_parameter("robot_base_frame").value,
            size_weight=float(node.get_parameter("size_weight").value),
            blacklist_radius=float(node.get_parameter("blacklist_radius").value),
            completion_patience=int(node.get_parameter("completion_patience").value),
            min_goal_distance=float(node.get_parameter("min_goal_distance").value),
            goal_clearance=float(node.get_parameter("goal_clearance").value),
            reach_infl=float(node.get_parameter("reachability_inflation").value),
            max_snap_candidates=int(node.get_parameter("max_snap_candidates").value),
            map_stability_tol=int(node.get_parameter("map_stability_tol").value),
            goal_timeout=float(node.get_parameter("goal_timeout").value),
            nbrs=nbrs,
        )


# ======================================================================
# Wahrnehmung
# ======================================================================
@dataclass
class DetectionResult:
    """Ergebnis einer Kartenauswertung.

    ``frontier`` ist die Zellmaske der Grenzzellen, ``n_cells`` ihre Anzahl,
    ``known`` die Zahl bekannter Zellen fuer die Stabilitaetspruefung, ``valid``
    die Liste anfahrbarer Ziele als ``(cx, cy, n, gx, gy)`` und
    ``skipped_clear`` die Zahl der wegen fehlenden Wandabstands verworfenen
    Cluster.
    """

    frontier: bytearray
    n_cells: int
    known: int
    valid: List[Tuple[float, float, int, float, float]]
    skipped_clear: int


class FrontierDetector:
    """Gewinnt aus einer Belegungskarte die anfahrbaren Grenzzellenziele.

    Der Ablauf verdickt zuerst die Hindernisse, markiert dann die Grenzzellen,
    gruppiert sie und waehlt je Gruppe eine Zielzelle mit ausreichendem
    Wandabstand.
    """

    def __init__(self, config: ExplorerConfig):
        self.config = config
        self._clear_offsets = None
        self._clear_offsets_rad = -1

    # ------------------------------------------------------------------
    def clearance_offsets(self, rad):
        """Kreisscheibe von Zelloffsets, nach Abstand aufsteigend, gecacht.

        Die aufsteigende Ordnung laesst die Freiraumpruefung im Ablehnungsfall
        frueh abbrechen, da eine stoerende Wand meist nah liegt.
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
        """Prueft, ob im Zellradius um ``(c0, r0)`` keine belegte Zelle liegt.

        Der Radius entspricht dem Aufblaehungsradius der Kostenkarte. Innerhalb
        dieses Radius drueckt der Regler den Roboter von der Wand weg, sodass
        die verbleibenden Fahrbefehle unter der Ansprechschwelle der gelernten
        Lauffortbewegung liegen und der Roboter stehen bleibt. Solche Ziele
        werden deshalb gar nicht erst erzeugt.

        Unbekannte Zellen zaehlen nicht als Hindernis. Andernfalls haette keine
        Grenzzelle je Freiraum. Die verbleibende Luecke gegen eine zum
        Wahlzeitpunkt noch unbekannte Wand schliesst die laufende Revalidierung.
        """
        if rad <= 0:
            return True
        for (dc, dr) in self.clearance_offsets(rad):
            rr = r0 + dr
            if rr < 0 or rr >= h:
                continue
            cc = c0 + dc
            if cc < 0 or cc >= w:
                continue
            if data[rr * w + cc] >= occ:
                return False
        return True

    def _clear_radius(self, res):
        """Wandabstand ``goal_clearance`` in Zellen der aktuellen Aufloesung."""
        if self.config.goal_clearance > 0.0:
            return int(math.ceil(self.config.goal_clearance / res))
        return 0

    # ------------------------------------------------------------------
    def detect(self, data, w, h, res, ox, oy) -> DetectionResult:
        """Wertet eine Karte vollstaendig aus und liefert die gueltigen Ziele."""
        occ = self.config.occupied_threshold
        max_occ = self.config.max_occ_nbr
        nbrs = self.config.nbrs

        frontier = bytearray(w * h)
        known = 0

        blocked = self._inflate(data, w, h, res, occ, nbrs)

        for row in range(h):
            base = row * w
            for col in range(w):
                v = data[base + col]
                if v < 0:
                    continue
                known += 1
                if v >= occ:
                    continue
                if blocked is not None and blocked[base + col]:
                    continue
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

        clusters = self._cluster(frontier, w, h, nbrs)
        valid, skipped_clear = self._filter(clusters, data, w, h, res, ox, oy, occ)

        return DetectionResult(frontier, n_cells, known, valid, skipped_clear)

    def _inflate(self, data, w, h, res, occ, nbrs):
        """Verdickt belegte Zellen um ``reachability_inflation``.

        Eine so markierte Zelle gilt bei der Detektion als Hindernis, sodass in
        zu engen Luecken keine Grenzzellen entstehen. Bei nicht positiver
        Aufblaehung entfaellt die Maske und das Verhalten bleibt unveraendert.
        """
        infl_cells = int(math.ceil(self.config.reach_infl / res)) if self.config.reach_infl > 0.0 else 0
        if infl_cells <= 0:
            return None
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
        return blocked

    def _cluster(self, frontier, w, h, nbrs):
        """Gruppiert zusammenhaengende Grenzzellen ueber eine Breitensuche."""
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
        return clusters

    def _filter(self, clusters, data, w, h, res, ox, oy, occ):
        """Waehlt je Cluster den Zentroid und die naechste anfahrbare Zelle.

        Verworfen werden Cluster unter der Mindestgroesse sowie solche, in denen
        keine Zelle den geforderten Wandabstand hat. Letztere sind Wandartefakte
        aus Winkelluecken des Laserscanners.
        """
        valid = []
        skipped_clear = 0
        clear_r = self._clear_radius(res)
        for comp in clusters:
            n = len(comp)
            if n < self.config.min_cluster_size:
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
            cand.sort(key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
            goal = None
            for (px, py, c, r) in cand[: self.config.max_snap_candidates]:
                if self.has_clearance(data, w, h, c, r, clear_r, occ):
                    goal = (px, py)
                    break
            if goal is None:
                skipped_clear += 1
                continue
            valid.append((cx, cy, n, goal[0], goal[1]))
        return valid, skipped_clear

    # ------------------------------------------------------------------
    def goal_validity(self, data, w, h, res, ox, oy, occ, active_goal) -> Optional[str]:
        """Prueft das aktive Ziel gegen die aktuelle Karte.

        Ein Grenzziel liegt an der Grenze zum Unbekannten. Waehrend der Anfahrt
        kann dort eine Wand sichtbar werden. Nav2 haelt dann am Ziel fest und der
        Roboter bleibt vor dem Kostenberg stehen. Gibt den Grund der
        Ungueltigkeit zurueck, sonst ``None``.
        """
        if active_goal is None:
            return None
        gx, gy = active_goal
        c = int((gx - ox) / res)
        r = int((gy - oy) / res)
        if c < 0 or c >= w or r < 0 or r >= h:
            return None
        if data[r * w + c] >= occ:
            return "Zielzelle ist inzwischen BELEGT"
        if not self.has_clearance(data, w, h, c, r, self._clear_radius(res), occ):
            return (f"Wandabstand unter goal_clearance "
                    f"({self.config.goal_clearance:.2f} m) gefallen")
        return None


# ======================================================================
# Zielsperren und Zielauswahl
# ======================================================================
class Blacklist:
    """Verwaltet gesperrte Zielorte und die Wiederholzaehler je Ort.

    Ein Ort wird gesperrt, sobald ein Ziel endgueltig scheitert. Der
    Wiederholzaehler auf einem Zehn-Zentimeter-Raster erlaubt einen einzigen
    erneuten Versuch, bevor gesperrt wird.
    """

    def __init__(self, radius: float, max_attempts: int):
        self.radius = radius
        self.max_attempts = max_attempts
        self._blocked: List[Tuple[float, float]] = []
        self._failures = {}

    def contains(self, gx, gy) -> bool:
        """True, wenn ``(gx, gy)`` innerhalb des Sperrradius eines Orts liegt."""
        r2 = self.radius ** 2
        for (bx, by) in self._blocked:
            if (gx - bx) ** 2 + (gy - by) ** 2 <= r2:
                return True
        return False

    def add(self, goal):
        """Sperrt einen Zielort."""
        if goal is not None:
            self._blocked.append(goal)

    def key(self, goal):
        """Bildet einen Zielort auf ein Zehn-Zentimeter-Raster ab."""
        if goal is None:
            return None
        return (round(goal[0], 1), round(goal[1], 1))

    def register_failure(self, key) -> int:
        """Zaehlt einen Fehlschlag am gerasterten Ort und gibt den Stand zurueck."""
        self._failures[key] = self._failures.get(key, 0) + 1
        return self._failures[key]

    def __len__(self):
        return len(self._blocked)


class GoalSelector:
    """Waehlt aus den gueltigen Zielen das guenstigste.

    Die Kosten eines Ziels sind seine Entfernung, vermindert um die mit
    ``size_weight`` gewichtete Clustergroesse. Zu nahe und gesperrte Ziele
    entfallen.
    """

    def __init__(self, config: ExplorerConfig, blacklist: Blacklist):
        self.config = config
        self.blacklist = blacklist

    def select_best(self, valid, robot):
        """Guenstigstes nicht gesperrtes Ziel mit ausreichendem Abstand."""
        if robot is None or not valid:
            return None
        rx, ry = robot
        best = None
        best_cost = None
        for (cx, cy, n, gx, gy) in valid:
            if self.blacklist.contains(gx, gy):
                continue
            dist = math.hypot(gx - rx, gy - ry)
            if dist < self.config.min_goal_distance:
                continue
            cost = dist - self.config.size_weight * n
            if best_cost is None or cost < best_cost:
                best_cost = cost
                yaw = math.atan2(gy - ry, gx - rx)
                best = (gx, gy, n, dist, cost, yaw)
        return best


# ======================================================================
# Auswertung
# ======================================================================
class RunMetrics:
    """Fuehrt Zeiten, gefahrene Distanz und Ereigniszaehler eines Laufs.

    Die Distanz wird aus der Odometrie im vollen Takt integriert, da die
    Kartenaktualisierung zu langsam ist, um die Strecke zu erfassen. Die
    Auswertung wird am Ende als eine uebernehmbare Zeile ausgegeben.
    """

    def __init__(self):
        self.goals_reached = 0
        self.n_revalidated = 0
        self.n_retries = 0
        self.n_timeouts = 0
        self.t_start = None
        self.t_explore_end = None
        self.t_home = None
        self.dist_total = 0.0
        self.dist_at_explore_end = None
        self.last_odom_xy = None
        self.rth_ok = None
        self.eval_printed = False

    def integrate_odom(self, position):
        """Addiert die Schrittweite; verwirft unplausible Spruenge."""
        xy = (position.x, position.y)
        if self.last_odom_xy is not None:
            dx = xy[0] - self.last_odom_xy[0]
            dy = xy[1] - self.last_odom_xy[1]
            step = math.hypot(dx, dy)
            if step < 0.5:
                self.dist_total += step
        self.last_odom_xy = xy

    def mark_start(self, now):
        """Setzt den Startzeitpunkt und den Distanzzaehler zurueck."""
        self.t_start = now
        self.dist_total = 0.0

    def mark_explore_end(self, now):
        """Friert Zeit und Distanz am Ende der Explorationsphase ein."""
        self.t_explore_end = now
        self.dist_at_explore_end = self.dist_total

    def mark_home(self, now, ok):
        """Haelt Ankunftszeit und Erfolg der Rueckkehr fest."""
        self.t_home = now
        self.rth_ok = ok

    def count_reached(self):
        self.goals_reached += 1

    def count_revalidate(self):
        self.n_revalidated += 1

    def count_retry(self):
        self.n_retries += 1

    def count_timeout(self):
        self.n_timeouts += 1

    def print_eval(self, logger, blacklist_size, config: ExplorerConfig):
        """Gibt die Laufmetriken einmalig aus, inklusive der CSV-Zeile."""
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

        L = logger
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
        L.info(f"[EVAL] Blacklist-Eintraege    : {blacklist_size}")
        L.info(f"[EVAL] Ziel-Revalidierungen(C): {self.n_revalidated}")
        L.info(f"[EVAL] Ziel-Retries (D)       : {self.n_retries}")
        L.info(f"[EVAL] Ziel-Timeouts (A)      : {self.n_timeouts}")
        L.info(f"[EVAL] Konfig                 : goal_clearance={config.goal_clearance} "
               f"min_goal_distance={config.min_goal_distance} size_weight={config.size_weight}")
        L.info(
            "[EVAL] CSV;ziele;t_expl;t_rth;t_ges;d_expl;d_rth;d_ges;rth;blacklist;reval;retry;timeout;clearance"
        )
        L.info(
            f"[EVAL] CSV;{self.goals_reached};{f(t_expl)};{f(t_rth)};{f(t_ges)};"
            f"{f(d_expl, 2)};{f(d_rth, 2)};{f(d_ges, 2)};{rth};{blacklist_size};"
            f"{self.n_revalidated};{self.n_retries};{self.n_timeouts};{config.goal_clearance}"
        )
        L.info("=" * 64)
        L.info("[EVAL] HINWEIS: Karte JETZT sichern, solange SLAM laeuft:")
        L.info("[EVAL]   ros2 run nav2_map_server map_saver_cli -f /root/maps/<name> -t /map")
        L.info("=" * 64)


# ======================================================================
# Anzeige
# ======================================================================
class MarkerPublisher:
    """Erzeugt die RViz-Anzeige aus Grenzzellen, Zentroiden und aktivem Ziel.

    Die Farben sind bewusst von der Kostenkarte verschieden. Gruen sind die
    Grenzzellen, orange die Zentroide, rot der beste Kandidat, gold das aktive
    Ziel.
    """

    def __init__(self, publisher):
        self.pub = publisher

    def publish(self, header, frontier, w, h, res, ox, oy, valid, best, active_goal):
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
        cells.color = ColorRGBA(r=0.1, g=1.0, b=0.2, a=0.9)
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
        cent.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=1.0)
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
            goal.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
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
        if active_goal is not None:
            ax, ay = active_goal
            act.action = Marker.ADD
            act.scale.x = 0.2
            act.scale.y = 0.2
            act.scale.z = 0.8
            act.color = ColorRGBA(r=1.0, g=0.85, b=0.0, a=0.85)
            act.pose.position.x = ax
            act.pose.position.y = ay
            act.pose.position.z = 0.4
            act.pose.orientation.w = 1.0
        else:
            act.action = Marker.DELETE
        arr.markers.append(act)

        self.pub.publish(arr)


# ======================================================================
# Zustandsautomat und ROS-Anbindung
# ======================================================================
class FrontierExplorerNode(Node):
    """Haelt den Zustandsautomaten und die Anbindung an ROS zusammen.

    Der Knoten abonniert Karte und Odometrie, fragt die Pose ueber TF ab und
    steuert die Zielvergabe an Nav2. Die fachliche Arbeit delegiert er an
    Detektor, Zielauswahl, Sperrliste, Metrik und Anzeige.
    """

    def __init__(self):
        super().__init__("frontier_explorer")

        self.config = ExplorerConfig.from_node(self)
        self.detector = FrontierDetector(self.config)
        self.blacklist = Blacklist(self.config.blacklist_radius, MAX_GOAL_ATTEMPTS)
        self.selector = GoalSelector(self.config, self.blacklist)
        self.metrics = RunMetrics()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.state = STATE_IDLE
        self.active_goal = None
        self.current_goal_handle = None
        self.empty_count = 0
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

        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(
            OccupancyGrid, self.config.map_topic, self.map_cb, map_qos
        )
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 20)
        self.pub = self.create_publisher(MarkerArray, self.config.marker_topic, 1)
        self.markers = MarkerPublisher(self.pub)

        self.timeout_timer = self.create_timer(1.0, self.check_goal_timeout)

        self.get_logger().info(
            f"[3d] Autonomiezyklus aktiv: sub '{self.config.map_topic}', pub '{self.config.marker_topic}'; "
            f"size_weight={self.config.size_weight}, blacklist_radius={self.config.blacklist_radius}, "
            f"min_goal_distance={self.config.min_goal_distance}, goal_clearance={self.config.goal_clearance}, "
            f"max_goal_attempts={MAX_GOAL_ATTEMPTS}, revalidierung=an, "
            f"completion_patience={self.config.completion_patience}, "
            f"map_stability_tol={self.config.map_stability_tol}, goal_timeout={self.config.goal_timeout}s, "
            f"frames {self.config.global_frame}->{self.config.robot_base_frame}"
        )

    # ------------------------------------------------------------------
    def map_cb(self, msg: OccupancyGrid):
        """Wertet eine Karte aus, aktualisiert Anzeige und Zielvergabe."""
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

        result = self.detector.detect(data, w, h, res, ox, oy)

        if self.last_known_count is None:
            self.map_is_stable = False
        else:
            self.map_is_stable = abs(result.known - self.last_known_count) <= self.config.map_stability_tol
        self.last_known_count = result.known

        robot = None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.config.global_frame, self.config.robot_base_frame, Time()
            )
            robot = (tf.transform.translation.x, tf.transform.translation.y)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF {self.config.global_frame}->{self.config.robot_base_frame} nicht verfuegbar: {e}",
                throttle_duration_sec=5.0,
            )

        self.latest_valid = result.valid
        self.latest_robot = robot

        if self.home_pose is None and robot is not None:
            self.home_pose = robot
            self.metrics.mark_start(self.get_clock().now())
            self.get_logger().info(
                f"[3d] Home-Pose gemerkt (Startpose): ({robot[0]:.2f}, {robot[1]:.2f})."
            )

        best = self.selector.select_best(result.valid, robot)

        sizes = sorted((v[2] for v in result.valid), reverse=True)
        self.get_logger().info(
            f"[{self.state}] Frontiers: {result.n_cells} Zellen | {len(result.valid)} gueltig | "
            f"{result.skipped_clear} verworfen (Clearance) | "
            f"Groessen={sizes[:5]} | Karte={'stabil' if self.map_is_stable else 'waechst'} | "
            f"Blacklist={len(self.blacklist)} | Ziele erreicht={self.metrics.goals_reached}"
        )

        self.markers.publish(msg.header, result.frontier, w, h, res, ox, oy,
                             result.valid, best, self.active_goal)

        if self.state == STATE_NAVIGATING and not self.goal_invalidated:
            reason = self.detector.goal_validity(
                data, w, h, res, ox, oy, self.config.occupied_threshold, self.active_goal
            )
            if reason is not None:
                self.invalid_reason = reason
                self.abort_goal_invalid()
                return

        if self.state == STATE_IDLE:
            self.try_select_and_send()

    # ------------------------------------------------------------------
    def odom_cb(self, msg):
        """Integriert die gefahrene Distanz aus der Odometrie."""
        self.metrics.integrate_odom(msg.pose.pose.position)

    # ------------------------------------------------------------------
    def abort_goal_invalid(self):
        """Bricht ein entwertetes Ziel ab, ohne den Ort zu sperren.

        Der Cluster bleibt gueltig. Die Wand ist nun bekannt, sodass die naechste
        Auswertung die wandnahen Zellen von selbst verwirft. Eine Sperre wuerde
        die Region unnoetig aufgeben.
        """
        self.get_logger().warn(
            f"[C] Ziel {self.active_goal} ist ungueltig geworden: {self.invalid_reason} "
            f"-> abbrechen, Neuwahl (KEINE Blacklist)."
        )
        self.goal_invalidated = True
        self.metrics.count_revalidate()
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
        else:
            self.state = STATE_IDLE

    def try_select_and_send(self):
        """Waehlt im Leerlauf ein Ziel oder leitet den Abschluss ein."""
        if self.shutting_down or self.state != STATE_IDLE:
            return
        best = self.selector.select_best(self.latest_valid, self.latest_robot)
        if best is None:
            if self.latest_robot is None:
                return
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
                f"({self.empty_count}/{self.config.completion_patience})."
            )
            if self.empty_count >= self.config.completion_patience:
                self.get_logger().info(
                    f"[3c] ===> EXPLORATION FERTIG: keine Frontiers mehr, Karte stabil ueber "
                    f"{self.config.completion_patience} Updates. {self.metrics.goals_reached} Ziele erreicht. "
                    f"Starte Return-to-Home."
                )
                self.metrics.mark_explore_end(self.get_clock().now())
                self.start_return_home()
            return
        self.empty_count = 0
        self.send_goal(best)

    def send_goal(self, best):
        """Sendet ein Explorationsziel an Nav2 und wechselt nach NAVIGATING."""
        gx, gy, n, dist, cost, yaw = best
        goal_msg = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self.config.global_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = gx
        ps.pose.position.y = gy
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        goal_msg.pose = ps

        self.state = STATE_NAVIGATING
        self.active_goal = (gx, gy)
        self.goal_start_time = self.get_clock().now()
        self.goal_timed_out = False
        self.get_logger().info(
            f"[3b] Ziel #{self.metrics.goals_reached + 1}: ({gx:.2f}, {gy:.2f}), "
            f"Cluster {n} Zellen, Dist {dist:.2f} m -> NAVIGATING"
        )
        fut = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        fut.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        """Behandelt die Annahme oder Ablehnung eines Explorationsziels."""
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn(f"[3b] Ziel ABGELEHNT -> Blacklist {self.active_goal}.")
            if self.active_goal is not None:
                self.blacklist.add(self.active_goal)
            self.current_goal_handle = None
            self.state = STATE_IDLE
            return
        self.current_goal_handle = gh
        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self.result_cb)

    def feedback_cb(self, feedback_msg):
        """Meldet die Restdistanz waehrend der Fahrt."""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"[{self.state}] unterwegs... Restdistanz {fb.distance_remaining:.2f} m",
            throttle_duration_sec=3.0,
        )

    def result_cb(self, future):
        """Wertet das Ergebnis eines Explorationsziels aus.

        Erfolg fuehrt zum naechsten Ziel. Ein durch Revalidierung oder Timeout
        ausgeloester Abbruch wird gesondert behandelt. Ein Fehlschlag erhaelt
        einen einzigen Wiederholversuch, bevor der Ort gesperrt wird. Der
        Wiederholversuch faengt das Wettrennen zwischen der direkt gelesenen
        Karte und der spaeter aktualisierten Kostenkarte von Nav2 ab.
        """
        status = future.result().status
        self.current_goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.metrics.count_reached()
            self.get_logger().info(
                f"[3b] ===> ERREICHT (#{self.metrics.goals_reached}) -> naechstes Ziel."
            )
            self.state = STATE_IDLE
        elif status == GoalStatus.STATUS_CANCELED:
            if self.goal_invalidated:
                self.goal_invalidated = False
                self.get_logger().info("[C] Ziel abgebrochen (Karte hat es entwertet). Neuwahl.")
                self.state = STATE_IDLE
                return
            if self.goal_timed_out:
                self.goal_timed_out = False
                if self.active_goal is not None:
                    self.blacklist.add(self.active_goal)
                self.get_logger().warn(
                    f"[timeout] Ziel verworfen -> Blacklist ({len(self.blacklist)}). Naechstes Ziel."
                )
                self.state = STATE_IDLE
            else:
                self.get_logger().warn("[3b] Ziel canceled (Shutdown).")
                self.state = STATE_IDLE
        else:
            key = self.blacklist.key(self.active_goal)
            fails = self.blacklist.register_failure(key)
            if fails < MAX_GOAL_ATTEMPTS:
                self.metrics.count_retry()
                self.get_logger().warn(
                    f"[D] FEHLGESCHLAGEN (Status {status}) bei {self.active_goal} "
                    f"-- Versuch {fails}/{MAX_GOAL_ATTEMPTS}, KEINE Blacklist, Neuwahl."
                )
            else:
                if self.active_goal is not None:
                    self.blacklist.add(self.active_goal)
                self.get_logger().warn(
                    f"[3b] FEHLGESCHLAGEN (Status {status}) nach {fails} Versuchen "
                    f"-> Blacklist {self.active_goal} ({len(self.blacklist)})."
                )
            self.state = STATE_IDLE

    # ------------------------------------------------------------------
    def check_goal_timeout(self):
        """Verwirft ein Explorationsziel, das zu lange aktiv bleibt."""
        if self.shutting_down or self.state != STATE_NAVIGATING:
            return
        if self.goal_timed_out or self.goal_invalidated or self.goal_start_time is None:
            return
        elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
        if elapsed >= self.config.goal_timeout:
            self.get_logger().warn(
                f"[timeout] Ziel {self.active_goal} seit {elapsed:.0f}s aktiv "
                f"(> {self.config.goal_timeout:.0f}s) -> cancele."
            )
            self.goal_timed_out = True
            self.metrics.count_timeout()
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()

    # ------------------------------------------------------------------
    def start_return_home(self):
        """Sendet die gemerkte Startpose als Ziel und wechselt nach RETURNING."""
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
        ps.header.frame_id = self.config.global_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = hx
        ps.pose.position.y = hy
        ps.pose.orientation.w = 1.0
        goal_msg.pose = ps
        self.get_logger().info(
            f"[3d] ===> RETURN-TO-HOME (Versuch {self.home_attempts + 1}/{MAX_HOME_ATTEMPTS}): "
            f"fahre zur Startpose ({hx:.2f}, {hy:.2f})."
        )
        fut = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        fut.add_done_callback(self.home_response_cb)

    def home_response_cb(self, future):
        """Behandelt die Annahme oder Ablehnung des Rueckkehrziels."""
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
        """Wertet das Ergebnis der Rueckkehr aus und schliesst den Lauf ab."""
        status = future.result().status
        self.current_goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.state = STATE_DONE
            self.active_goal = None
            self.metrics.mark_home(self.get_clock().now(), True)
            self.get_logger().info(
                f"[3d] ===> HOME ERREICHT: zurueck an der Startpose. "
                f"{self.metrics.goals_reached} Frontier-Ziele erkundet. Autonomiezyklus komplett."
            )
            self.metrics.print_eval(self.get_logger(), len(self.blacklist), self.config)
            return
        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("[3d] Home-Ziel canceled (Shutdown).")
            self.state = STATE_DONE
            self.active_goal = None
            return
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
            self.metrics.mark_home(self.get_clock().now(), False)
            self.metrics.print_eval(self.get_logger(), len(self.blacklist), self.config)

    # ------------------------------------------------------------------
    def cancel_active_goal(self):
        """Bricht beim Beenden ein laufendes Ziel ab und gibt Teilmetriken aus."""
        self.shutting_down = True
        self.metrics.print_eval(self.get_logger(), len(self.blacklist), self.config)
        if self.current_goal_handle is not None:
            self.get_logger().info("[3b] Beende -> cancele laufendes Nav2-Ziel...")
            try:
                fut = self.current_goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
            except Exception as e:
                self.get_logger().warn(f"[3b] Cancel fehlgeschlagen: {e}")


def main():
    rclpy.init(args=sys.argv)
    node = FrontierExplorerNode()
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

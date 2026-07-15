#!/usr/bin/env python3
"""eval_probe.py -- EIN passives Mess-Terminal fuer die Evaluierungslaeufe.

Startet den Explorer NICHT. Der Explorer laeuft wie gewohnt eigenstaendig in
seinem eigenen Terminal und druckt dort seine [EVAL]-Metriken. Dieses Werkzeug
laeuft PARALLEL in einem zusaetzlichen Terminal und misst alles, was der Explorer
nicht kennt -- und gibt am Ende EINEN Block aus, den du direkt hierher zurueck-
kopieren kannst.

Vereint drei bisher getrennte Werkzeuge:
  * tilt_tf_probe   -> Rumpf-Roll/Nick (Grad) + map->odom-Drift
  * cmd_odom_probe  -> Befehl vs. Ist-Bewegung, Haenger-Erkennung
  * check_map_shear -> Scherwinkel der Karte (Algorithmus 1:1 uebernommen)

WORKFLOW PRO LAUF (ein Terminal zusaetzlich, Explorer bleibt separat):
  Terminal 5 (wie immer):  python3 /root/repo/frontier/frontier_explorer.py --ros-args -p use_sim_time:=true
  Terminal 6 (dieses):     python3 /root/repo/tools/eval_probe.py neu_1 --ros-args -p use_sim_time:=true

  Laufen lassen. Wenn der Explorer "HOME ERREICHT" meldet -> hier Strg-C.
  Das Werkzeug speichert die Karte (aus /map, kein map_saver noetig), misst den
  Shear und druckt den Sammelblock. Diesen Block komplett kopieren.

  Baseline-Serie: nur beim EXPLORER die Extra-Args setzen. Der Name hier ist frei
  waehlbar (z.B. base_1) und dient nur der Beschriftung.

Der Lauf-Name ist das ERSTE Argument (vor --ros-args). Fehlt er, heisst der Lauf "lauf".
Karten + Sammel-CSV landen in /root/maps/eval/ (auf dem Host: ~/welt2_slam/maps/eval/).
"""
import math
import os
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

LASER_Z = 0.12          # m -- LiDAR-Hoehe ueber base (aus play_go2_ros_scan.py)
MIN_RATE_HZ = 20.0      # darunter ist die Neigungs-AMPLITUDE nicht belastbar
CMD_ON = 0.05           # ab hier gilt "es wird etwas befohlen"
IST_OFF = 0.03          # darunter gilt "der Roboter tut nichts"
OUTDIR = "/root/maps/eval"
SHEAR_OCC = 50          # exakt wie check_map_shear.py


# ============================================================
# Lauf-Namen aus argv fischen (erstes Nicht-Flag vor --ros-args)
# ============================================================
def parse_run_name(argv):
    for a in argv[1:]:
        if a == "--ros-args":
            break
        if not a.startswith("-"):
            return a
    return "lauf"


# ============================================================
# Quaternion -> RPY
# ============================================================
def quat_to_rpy(x, y, z, w):
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


# ============================================================
# Shear-Analyse -- Algorithmus 1:1 aus check_map_shear.py,
# nur auf ein OccupancyGrid-Array statt eine PGM-Datei angewandt.
# grid: bytes/list mit Werten 0..100 frei..belegt, -1 unbekannt.
# Umsetzung auf PGM-Konvention: belegt(>=OCC_GRID) -> "dunkel" (< OCC in PGM).
# check_map_shear sucht die erste Zelle < OCC(=50 grau) = frei/hell? Nein:
# PGM aus map_saver: 0=belegt(schwarz), 254=frei(weiss), 205=unbekannt.
# "< OCC=50" trifft also nur BELEGTE Zellen (schwarz). left(r)=erste belegte Spalte.
# Aequivalent im Grid: Wert >= 65 (belegt).
# ============================================================
def shear_from_grid(grid, w, h):
    OCC_GRID = 65

    # WICHTIG: check_map_shear liest die gespeicherte PGM (top-down). Das
    # OccupancyGrid ist bottom-up. Damit die VORZEICHEN identisch zu
    # check_map_shear sind, spiegeln wir hier auf top-down, bevor wir messen.
    def cell(r, c):
        return grid[(h - 1 - r) * w + c]

    def is_wall(r, c):
        return cell(r, c) >= OCC_GRID

    def left(r):
        for c in range(w):
            if is_wall(r, c):
                return c
        return None

    def top(c):
        for r in range(h):
            if is_wall(r, c):
                return r
        return None

    def med(fn, idx):
        vals = [x for x in (fn(i) for i in idx) if x is not None]
        return statistics.median(vals) if vals else None

    rt, rb = range(int(h * .2), int(h * .3)), range(int(h * .7), int(h * .8))
    ct, cb = range(int(w * .2), int(w * .3)), range(int(w * .7), int(w * .8))
    lt, lb = med(left, rt), med(left, rb)
    tt, tb = med(top, ct), med(top, cb)

    res = {"w": w, "h": h, "links": None, "oben": None}
    if None not in (lt, lb):
        res["links"] = math.degrees(math.atan2(lb - lt, rb.start - rt.start))
    if None not in (tt, tb):
        res["oben"] = math.degrees(math.atan2(tb - tt, cb.start - ct.start))
    return res


def write_pgm(path, grid, w, h):
    """Karte im map_saver-kompatiblen PGM ablegen (0=belegt,254=frei,205=unbekannt)."""
    buf = bytearray(w * h)
    for i in range(w * h):
        v = grid[i]
        if v < 0:
            buf[i] = 205
        elif v >= 65:
            buf[i] = 0
        else:
            buf[i] = 254
    # PGM y-Achse ist top-down; OccupancyGrid ist bottom-up -> Zeilen spiegeln
    rows = [buf[r * w:(r + 1) * w] for r in range(h)]
    rows.reverse()
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        for row in rows:
            f.write(bytes(row))


class EvalProbe(Node):
    def __init__(self, run_name):
        super().__init__("eval_probe")
        self.run_name = run_name

        # --- Neigung (aus /odom-Orientierung) ---
        self.n_odom = 0
        self.t_first = self.t_last = None
        self.roll_min = self.roll_max = None
        self.pitch_min = self.pitch_max = None
        self.roll_sq = self.pitch_sq = 0.0
        self.moving = 0

        # --- Distanz (aus /odom-Position) ---
        self.dist = 0.0
        self.last_xy = None

        # --- Haenger (Befehl vs. Ist) ---
        self.cmd_vx = self.cmd_wz = 0.0
        self.ist_vx = self.ist_wz = 0.0
        self.stall_since = None
        self.stall_events = 0
        self.stall_max = 0.0
        self._in_stall = False

        # --- map->odom Drift ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.mo_tx, self.mo_ty, self.mo_yaw = [], [], []
        self.mo_fails = 0

        # --- letzte Karte ---
        self.grid = None
        self.grid_w = self.grid_h = 0
        self.grid_res = 0.05

        self.create_subscription(Odometry, "/odom", self.odom_cb, 50)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)
        self.create_subscription(OccupancyGrid, "/map", self.map_cb, 1)
        self.create_timer(0.1, self.tf_tick)
        self.create_timer(0.2, self.stall_tick)

        self.get_logger().info(
            f"eval_probe aktiv (Lauf '{run_name}'). Explorer separat starten und FAHREN lassen. "
            f"Am Ende (HOME ERREICHT) hier Strg-C."
        )

    # -------- Callbacks --------
    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        roll, pitch, _ = quat_to_rpy(q.x, q.y, q.z, q.w)
        roll, pitch = math.degrees(roll), math.degrees(pitch)
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.t_first is None:
            self.t_first = t
        self.t_last = t
        self.n_odom += 1
        self.roll_min = roll if self.roll_min is None else min(self.roll_min, roll)
        self.roll_max = roll if self.roll_max is None else max(self.roll_max, roll)
        self.pitch_min = pitch if self.pitch_min is None else min(self.pitch_min, pitch)
        self.pitch_max = pitch if self.pitch_max is None else max(self.pitch_max, pitch)
        self.roll_sq += roll * roll
        self.pitch_sq += pitch * pitch

        v = msg.twist.twist
        self.ist_vx, self.ist_wz = v.linear.x, v.angular.z
        if abs(v.linear.x) > 0.05 or abs(v.angular.z) > 0.05:
            self.moving += 1

        p = msg.pose.pose.position
        xy = (p.x, p.y)
        if self.last_xy is not None:
            step = math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
            if step < 0.5:
                self.dist += step
        self.last_xy = xy

    def cmd_cb(self, msg):
        self.cmd_vx, self.cmd_wz = msg.linear.x, msg.angular.z

    def map_cb(self, msg):
        self.grid = msg.data
        self.grid_w = msg.info.width
        self.grid_h = msg.info.height
        self.grid_res = msg.info.resolution

    def tf_tick(self):
        try:
            tr = self.tf_buffer.lookup_transform("map", "odom", Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.mo_fails += 1
            return
        t = tr.transform.translation
        q = tr.transform.rotation
        _, _, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        self.mo_tx.append(t.x)
        self.mo_ty.append(t.y)
        self.mo_yaw.append(math.degrees(yaw))

    def stall_tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        befohlen = abs(self.cmd_vx) > CMD_ON or abs(self.cmd_wz) > CMD_ON
        passiert = abs(self.ist_vx) > IST_OFF or abs(self.ist_wz) > IST_OFF
        if befohlen and not passiert:
            if self.stall_since is None:
                self.stall_since = t
                if not self._in_stall:
                    self.stall_events += 1
                    self._in_stall = True
            self.stall_max = max(self.stall_max, t - self.stall_since)
        else:
            self.stall_since = None
            self._in_stall = False

    def rate_hz(self):
        if self.t_first is None or self.t_last is None or self.t_last <= self.t_first:
            return 0.0
        return (self.n_odom - 1) / (self.t_last - self.t_first)

    # -------- Abschlussbericht: EIN Copy-Paste-Block --------
    def final(self):
        hz = self.rate_hz()
        # Karte speichern + Shear
        shear = None
        map_path = "n/a"
        if self.grid is not None and self.grid_w > 0:
            os.makedirs(OUTDIR, exist_ok=True)
            map_path = os.path.join(OUTDIR, self.run_name + ".pgm")
            try:
                write_pgm(map_path, self.grid, self.grid_w, self.grid_h)
            except Exception as e:
                map_path = f"FEHLER beim Speichern: {e}"
            try:
                shear = shear_from_grid(self.grid, self.grid_w, self.grid_h)
            except Exception as e:
                shear = {"err": str(e)}

        def g(v, nk=2):
            return "n/a" if v is None else f"{v:.{nk}f}"

        p_amp = None if self.pitch_min is None else (self.pitch_max - self.pitch_min) / 2.0
        r_amp = None if self.roll_min is None else (self.roll_max - self.roll_min) / 2.0
        p_ext = None if self.pitch_min is None else max(abs(self.pitch_min), abs(self.pitch_max))
        r_ext = None if self.roll_min is None else max(abs(self.roll_min), abs(self.roll_max))
        mo_dtx = (max(self.mo_tx) - min(self.mo_tx)) * 100 if self.mo_tx else None
        mo_dty = (max(self.mo_ty) - min(self.mo_ty)) * 100 if self.mo_ty else None
        mo_dyaw = (max(self.mo_yaw) - min(self.mo_yaw)) if self.mo_yaw else None

        sl = so = sw = sh = None
        if shear and "err" not in shear:
            sl, so, sw, sh = shear["links"], shear["oben"], shear["w"], shear["h"]

        L = []
        L.append("")
        L.append("========================= EVAL-PROBE BLOCK =========================")
        L.append(f"# Lauf: {self.run_name}   ({self.n_odom} odom @ {hz:.1f} Hz, davon {self.moving} in Bewegung)")
        L.append(f"# Karte gespeichert: {map_path}")
        L.append("")
        L.append("--- Neigung (Rumpf, aus /odom) ---")
        L.append(f"Nick : {g(self.pitch_min)} .. {g(self.pitch_max)} deg  (Amplitude {g(p_amp)})")
        L.append(f"Roll : {g(self.roll_min)} .. {g(self.roll_max)} deg  (Amplitude {g(r_amp)})")
        if hz and hz < MIN_RATE_HZ:
            L.append(f"[!] {hz:.1f} Hz < {MIN_RATE_HZ:.0f} Hz -> Amplitude untere Schranke, nicht belastbar.")
        L.append("")
        L.append("--- map->odom Drift (Ground-Truth-Odometrie => sollte ~konstant sein) ---")
        if self.mo_tx:
            L.append(f"tx-Spanne {g(mo_dtx)} cm | ty-Spanne {g(mo_dty)} cm | yaw-Spanne {g(mo_dyaw)} deg | Samples {len(self.mo_tx)}")
        else:
            L.append(f"keine Transform ({self.mo_fails} Fehlversuche) -- lief SLAM?")
        L.append("")
        L.append("--- Haenger (Befehl != Ist-Bewegung) ---")
        L.append(f"Ereignisse: {self.stall_events} | laengster: {g(self.stall_max, 1)} s (Wall-Time)")
        L.append("")
        L.append("--- Karten-Shear (Algorithmus wie check_map_shear) ---")
        if shear is None:
            L.append("keine Karte empfangen.")
        elif "err" in shear:
            L.append(f"Fehler: {shear['err']}")
        else:
            L.append(f"Groesse: {sw} x {sh} px (~{sw*0.05:.2f} x {sh*0.05:.2f} m)")
            L.append(f"Linke Wand: {g(sl, 1)} deg | Obere Wand: {g(so, 1)} deg  (ideal ~0)")
        L.append("")
        L.append("--- CSV (eine Zeile; Explorer-[EVAL]-CSV separat daneben legen) ---")
        L.append("name;odom_hz;nick_min;nick_max;roll_min;roll_max;mo_tx_cm;mo_ty_cm;mo_yaw_deg;stalls;stall_max_s;shear_links;shear_oben;karte_b;karte_h")
        L.append(
            f"{self.run_name};{g(hz,1)};{g(self.pitch_min)};{g(self.pitch_max)};"
            f"{g(self.roll_min)};{g(self.roll_max)};{g(mo_dtx)};{g(mo_dty)};{g(mo_dyaw)};"
            f"{self.stall_events};{g(self.stall_max,1)};{g(sl,1)};{g(so,1)};"
            f"{sw if sw else 'n/a'};{sh if sh else 'n/a'}"
        )
        L.append("====================================================================")
        L.append("")
        block = "\n".join(L)
        print(block, flush=True)

        # zusaetzlich an Sammel-CSV anhaengen
        try:
            os.makedirs(OUTDIR, exist_ok=True)
            csv = os.path.join(OUTDIR, "probe_ergebnisse.csv")
            new = not os.path.exists(csv)
            with open(csv, "a") as f:
                if new:
                    f.write("name;odom_hz;nick_min;nick_max;roll_min;roll_max;mo_tx_cm;mo_ty_cm;mo_yaw_deg;stalls;stall_max_s;shear_links;shear_oben;karte_b;karte_h\n")
                f.write(
                    f"{self.run_name};{g(hz,1)};{g(self.pitch_min)};{g(self.pitch_max)};"
                    f"{g(self.roll_min)};{g(self.roll_max)};{g(mo_dtx)};{g(mo_dty)};{g(mo_dyaw)};"
                    f"{self.stall_events};{g(self.stall_max,1)};{g(sl,1)};{g(so,1)};"
                    f"{sw if sw else 'n/a'};{sh if sh else 'n/a'}\n"
                )
            print(f"(auch angehaengt an {csv})", flush=True)
        except Exception as e:
            print(f"(Sammel-CSV nicht geschrieben: {e})", flush=True)


def main():
    run_name = parse_run_name(sys.argv)
    rclpy.init(args=sys.argv)
    node = EvalProbe(run_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.final()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

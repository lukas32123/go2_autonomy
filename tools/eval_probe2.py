#!/usr/bin/env python3
"""eval_probe2 -- ERGAENZUNG zu eval_probe.py, KEIN Ersatz.

eval_probe.py laeuft unveraendert weiter und schreibt weiter probe_ergebnisse.csv.
Dieser Knoten laeuft parallel und ist rein passiv: er abonniert nur, er publiziert
nichts und greift in keinen Regelkreis ein. Ein Lauf kann durch ihn nicht anders
ausgehen als ohne ihn.

Was er zusaetzlich liefert:
  1. /rosout mitschneiden  -> <name>_explorer.log  (Verlustsicherung, siehe 20.07.)
  2. Ereignisse zaehlen    -> Planerausfaelle, spin/wait/backup, Costmap-Leerungen,
                              verworfene Kartennachrichten, Clearance-Verwuerfe
  3. Planerausfaelle mit KOORDINATEN erfassen und den Abschnitt klassifizieren
     (gerade Fluranfahrt / Eckanfahrt) -- das ist der Datensatz fuer Kapitel 5
  4. /map selbst validieren, jede Teilpruefung EINZELN zaehlen
  5. /collision_monitor_state  (Expose 5.3 "kollisionsfrei")
  6. /odom-Distanz als unabhaengiger Gegenwert zu d_ges des Explorers
  7. die [EVAL]-CSV-Zeile direkt aus /rosout parsen -> kein Abschreiben mehr
  8. alles in eval_gesamt.csv, EINE vollstaendige Zeile pro Lauf

Aufruf (im Container, NACH Nav2, VOR dem Explorer, eigenes Terminal):
    python3 /root/repo/tools/eval_probe2.py uni_neu_1 --ros-args -p use_sim_time:=true

Beenden: Strg-C. Sobald die [EVAL]-Zeile eingetroffen ist, meldet der Knoten das
ausdruecklich -- dann ist alles erfasst.

Reihenfolge am Laufende bleibt unveraendert:
    HOME ERREICHT abwarten -> Strg-C hier -> Strg-C im eval_probe (SLAM laeuft noch!)
    -> Explorer beenden -> herunterfahren
"""

import argparse
import csv
import math
import os
import re
import sys
import time
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from rcl_interfaces.msg import Log
from nav_msgs.msg import OccupancyGrid, Odometry

# collision_monitor_state ist optional: der Nachrichtentyp haengt an der Nav2-Version.
# Faellt der Import aus, laeuft alles andere trotzdem -- ein fehlendes Nebenmass darf
# keine Messreihe kosten.
try:
    from nav2_msgs.msg import CollisionMonitorState
    HAT_CM = True
except Exception:
    CollisionMonitorState = None
    HAT_CM = False

EIGENER_NAME = "eval_probe2"

# ----------------------------------------------------------------------------
# Szenen-Geometrie fuer die Abschnittsklassifikation.
# Konstanten aus isaac/patch_unigang.py -- NICHT aus dem Gedaechtnis, sondern
# dort abgelesen. Aendert sich die Szene, aendert sich das hier mit.
# ----------------------------------------------------------------------------
UNI_OUTER, UNI_WALL_T, UNI_NICHE_D = 30.0, 0.20, 0.40
UNI_CORR_WIDE, UNI_CORR_NARROW = 6.0, 3.0
_half = UNI_OUTER / 2.0
_back = _half - UNI_WALL_T / 2.0
UNI_PIER = _back - UNI_NICHE_D            # 14.5  Aussenpfeiler
UNI_CORE_X = UNI_PIER - UNI_CORR_NARROW   # 11.5  Kernpfeiler Ost/West
UNI_CORE_Y = UNI_PIER - UNI_CORR_WIDE     #  8.5  Kernpfeiler Nord/Sued


def flurabschnitt_uni(x, y):
    """N/S/O/W, wenn der Punkt eindeutig in einem Flurabschnitt liegt, sonst 'Ecke'."""
    if abs(y) > UNI_CORE_Y and abs(x) <= UNI_CORE_X:
        return "N" if y > 0 else "S"
    if abs(x) > UNI_CORE_X and abs(y) <= UNI_CORE_Y:
        return "O" if x > 0 else "W"
    return "Ecke"


def klassifiziere(szene, sx, sy, gx, gy):
    """('gerade'|'Ecke'|'n/a', Achsabweichung in Grad)."""
    winkel = math.degrees(math.atan2(abs(gy - sy), abs(gx - sx)))
    achse = min(winkel, 90.0 - winkel)
    if szene != "uni":
        return "n/a", achse
    a, b = flurabschnitt_uni(sx, sy), flurabschnitt_uni(gx, gy)
    if a == b and a != "Ecke":
        return "gerade", achse
    return "Ecke", achse


# ----------------------------------------------------------------------------
# Muster. Alle gegen echte Logzeilen aus den Laeufen vom 21.07. geprueft.
# ----------------------------------------------------------------------------
KO = r"\(\s*(-?[\d.]+),\s*(-?[\d.]+)\)"          # (x, y)

RE_ZIEL       = re.compile(r"\[3b\] Ziel #(\d+): " + KO + r", Cluster (\d+) Zellen, Dist ([\d.]+) m")
RE_ERREICHT   = re.compile(r"\[3b\] ===> ERREICHT \(#(\d+)\)")
RE_NAVIGIERE  = re.compile(r"Begin navigating from current location " + KO + r" to " + KO)
RE_PLANFEHLER = re.compile(r"GridBased plugin failed to plan from " + KO + r" to " + KO)
RE_RECOVERY   = re.compile(r"Running (spin|wait|backup)")
RE_CLEAR      = re.compile(r"Received request to clear entirely the (\S+)")
RE_MALFORMED  = re.compile(r"map message is malformed")
RE_UEBERSPR   = re.compile(r"Karte uebersprungen \(malformed\)")
RE_RATE       = re.compile(r"Planner loop missed its desired rate of [\d.]+ Hz\. "
                           r"Current loop rate is ([\d.]+)")
RE_RESIZE     = re.compile(r"Resizing costmap to (\d+) X (\d+)")
RE_FRONTIERS  = re.compile(r"Frontiers: (\d+) Zellen \| (\d+) gueltig \| (\d+) verworfen")
RE_EVAL_DATEN = re.compile(r"\[EVAL\] CSV;(\d+;.*)")   # Datenzeile, nicht die Kopfzeile
RE_HOME       = re.compile(r"HOME ERREICHT")

ZAEHLER_MUSTER = {
    "planer_fehler":   RE_PLANFEHLER,
    "spin":            re.compile(r"Running spin"),
    "wait":            re.compile(r"Running wait"),
    "backup":          re.compile(r"Running backup"),
    "costmap_clear":   RE_CLEAR,
    "map_malformed":   RE_MALFORMED,
    "map_uebersprungen": RE_UEBERSPR,
    "ziel_erreicht":   RE_ERREICHT,
    "reval_C":         re.compile(r"\[C\] "),
    "retry_D":         re.compile(r"\[D\] "),
    "timeout_A":       re.compile(r"\[timeout\]"),
}

GESAMT_SPALTEN = [
    "name", "szene", "gruppe",
    "ziele", "t_expl", "t_rth", "t_ges", "d_expl", "d_rth", "d_ges", "rth",
    "blacklist", "reval", "retry", "timeout", "clearance",
    "d_odom", "planer_fehler", "abschnitte", "gerade_fehler", "ecke_fehler",
    "spin", "wait", "backup", "costmap_clear",
    "map_malformed_nav2", "map_uebersprungen_expl",
    "map_n", "map_fehl_len", "map_fehl_res", "map_fehl_quat", "quat_abw_max",
    "collision_events", "clearance_verworfen_max", "planer_hz_min",
]


class EvalProbe2(Node):
    def __init__(self, opt):
        super().__init__(EIGENER_NAME)
        self.opt = opt
        self.t0 = time.time()

        self.zaehler = Counter()
        self.abschnitte = []      # {start, ziel, dist, typ, achse, fehler}
        self.offener_abschnitt = None
        self.ziele = []           # {nr, ziel, cluster, dist}
        self.eval_csv = None
        self.home = False

        # /map-Validierung -- Nachbau der Pruefung, die nav2_util::validateMsg macht.
        # Bewusst NICHT derselbe Code: weicht meine Zaehlung von Nav2 ab, ist genau
        # das die Information, die wir suchen.
        self.map_n = 0
        self.map_fehl = Counter()
        self.quat_abw_max = 0.0
        self.letzte_map = None

        self.d_odom = 0.0
        self.letzte_pos = None
        self.cm_events = 0
        self.clearance_max = 0
        self.planer_hz_min = None

        self.log_datei = open(os.path.join(opt.outdir, f"{opt.name}_explorer.log"),
                              "w", encoding="utf-8")

        self.create_subscription(Log, "/rosout", self.cb_rosout, 500)
        self.create_subscription(OccupancyGrid, "/map", self.cb_map, 1)
        self.create_subscription(Odometry, "/odom", self.cb_odom, 20)
        if HAT_CM:
            self.create_subscription(CollisionMonitorState, "/collision_monitor_state",
                                     self.cb_cm, 10)

        self.p(f"[{EIGENER_NAME}] Lauf '{opt.name}', Szene '{opt.szene}', "
               f"Ausgabe {opt.outdir}")
        if not HAT_CM:
            self.p("  ! nav2_msgs/CollisionMonitorState nicht importierbar -- "
                   "Kollisionszaehlung aus. Typ pruefen: "
                   "ros2 topic info /collision_monitor_state")
        self.p("  Warte auf den Explorer ...")

    # print statt get_logger: eine eigene Logzeile landete sonst wieder auf
    # /rosout und wuerde die eigene Zaehlung hochtreiben.
    def p(self, s):
        print(s, flush=True)

    # ------------------------------------------------------------------ /odom
    def cb_odom(self, m):
        p = m.pose.pose.position
        if self.letzte_pos is not None:
            d = math.hypot(p.x - self.letzte_pos[0], p.y - self.letzte_pos[1])
            if d < 1.0:                      # Sprung durch Reset ignorieren
                self.d_odom += d
        self.letzte_pos = (p.x, p.y)

    # -------------------------------------------------- /collision_monitor_state
    def cb_cm(self, m):
        # action_type 0 = DO_NOTHING. Alles andere ist ein Eingriff.
        if getattr(m, "action_type", 0) != 0:
            self.cm_events += 1
            self.p(f"  >> COLLISION_MONITOR greift ein: action_type={m.action_type} "
                   f"polygon={getattr(m, 'polygon_name', '?')}")

    # ------------------------------------------------------------------- /map
    def cb_map(self, m):
        self.map_n += 1
        w, h = m.info.width, m.info.height
        self.letzte_map = (w, h)
        fehler = []
        if w == 0 or h == 0 or len(m.data) != w * h:
            self.map_fehl["len"] += 1
            fehler.append(f"len={len(m.data)} != {w}*{h}")
        if not (m.info.resolution > 0.0) or not math.isfinite(m.info.resolution):
            self.map_fehl["res"] += 1
            fehler.append(f"resolution={m.info.resolution}")
        q = m.info.origin.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        abw = abs(norm - 1.0)
        self.quat_abw_max = max(self.quat_abw_max, abw)
        if abw > 1e-6:
            self.map_fehl["quat"] += 1
            fehler.append(f"|q|-1 = {abw:.3e}")
        if fehler:
            self.p(f"  >> MAP fehlerhaft (#{self.map_n}): {'; '.join(fehler)}")

    # ---------------------------------------------------------------- /rosout
    def cb_rosout(self, m):
        if m.name == EIGENER_NAME:
            return
        zeile = f"{m.stamp.sec}.{m.stamp.nanosec:09d} [{m.name}] {m.msg}"
        if m.name == "frontier_explorer":
            self.log_datei.write(zeile + "\n")

        for schluessel, regex in ZAEHLER_MUSTER.items():
            if regex.search(m.msg):
                self.zaehler[schluessel] += 1

        t = self.uhr()

        mm = RE_ZIEL.search(m.msg)
        if mm:
            nr, x, y, cl, d = mm.groups()
            self.ziele.append({"nr": int(nr), "ziel": (float(x), float(y)),
                               "cluster": int(cl), "dist": float(d)})
            self.p(f"  Ziel #{nr} ({x}, {y})  Cluster {cl}  Dist {d} m")

        mm = RE_NAVIGIERE.search(m.msg)
        if mm:
            sx, sy, gx, gy = (float(v) for v in mm.groups())
            typ, achse = klassifiziere(self.opt.szene, sx, sy, gx, gy)
            self.offener_abschnitt = {
                "t": t, "start": (sx, sy), "ziel": (gx, gy),
                "dist": math.dist((sx, sy), (gx, gy)),
                "typ": typ, "achse": achse, "fehler": 0,
            }
            self.abschnitte.append(self.offener_abschnitt)

        mm = RE_PLANFEHLER.search(m.msg)
        if mm:
            sx, sy, gx, gy = (float(v) for v in mm.groups())
            typ, achse = klassifiziere(self.opt.szene, sx, sy, gx, gy)
            d = math.dist((sx, sy), (gx, gy))
            if self.offener_abschnitt is not None:
                self.offener_abschnitt["fehler"] += 1
            if self.zaehler["planer_fehler"] == 1 or self.zaehler["planer_fehler"] % 5 == 0:
                self.p(f"  >> PLANERAUSFALL #{self.zaehler['planer_fehler']}: "
                       f"({sx:.2f}, {sy:.2f}) -> ({gx:.2f}, {gy:.2f}) | "
                       f"{d:.2f} m | {typ} | {achse:.2f} deg von der Achse")

        mm = RE_RECOVERY.search(m.msg)
        if mm:
            self.p(f"  >> RECOVERY: {mm.group(1)}")

        mm = RE_RATE.search(m.msg)
        if mm:
            hz = float(mm.group(1))
            self.planer_hz_min = hz if self.planer_hz_min is None else min(self.planer_hz_min, hz)

        mm = RE_FRONTIERS.search(m.msg)
        if mm:
            self.clearance_max = max(self.clearance_max, int(mm.group(3)))

        if RE_HOME.search(m.msg) and not self.home:
            self.home = True
            self.p("  >> HOME ERREICHT")

        mm = RE_EVAL_DATEN.search(m.msg)
        if mm:
            self.eval_csv = mm.group(1)
            self.p("\n  ================================================")
            self.p("  [EVAL]-Zeile erfasst -- alles Noetige ist gesichert.")
            self.p(f"  CSV;{self.eval_csv}")
            self.p("  Strg-C beendet und schreibt die Dateien.")
            self.p("  ================================================\n")

    def uhr(self):
        return time.time() - self.t0

    # ------------------------------------------------------------- Abschluss
    def abschluss(self):
        o = self.opt
        self.log_datei.close()

        zeilen = []
        zeilen.append("=" * 68)
        zeilen.append(f"EVAL-PROBE2  Lauf '{o.name}'  Szene '{o.szene}'")
        zeilen.append("=" * 68)

        zeilen.append("\n--- Ereignisse ---")
        if self.zaehler:
            for k, v in sorted(self.zaehler.items()):
                zeilen.append(f"  {k:22s} {v}")
        else:
            zeilen.append("  keine")
        if self.planer_hz_min is not None:
            zeilen.append(f"  {'planer_hz_min':22s} {self.planer_hz_min:.4f}")
        zeilen.append(f"  {'clearance_verworfen_max':22s} {self.clearance_max}")

        zeilen.append("\n--- Navigationsabschnitte ---")
        zeilen.append(f"  {'#':>2} {'Distanz':>8} {'Typ':>7} {'Achse':>7}  Planerausfaelle")
        ger_f = eck_f = 0
        for i, a in enumerate(self.abschnitte, 1):
            if a["fehler"]:
                if a["typ"] == "gerade":
                    ger_f += 1
                elif a["typ"] == "Ecke":
                    eck_f += 1
            zeilen.append(f"  {i:2d} {a['dist']:7.2f}m {a['typ']:>7} "
                          f"{a['achse']:6.2f}°  {a['fehler']}")
        ger = [a for a in self.abschnitte if a["typ"] == "gerade"]
        eck = [a for a in self.abschnitte if a["typ"] == "Ecke"]
        zeilen.append(f"  gerade: {len(ger)}, davon mit Ausfall {ger_f} | "
                      f"Ecke: {len(eck)}, davon mit Ausfall {eck_f}")

        zeilen.append("\n--- /map-Validierung (Nachbau von nav2_util::validateMsg) ---")
        zeilen.append(f"  Nachrichten           : {self.map_n}")
        zeilen.append(f"  letzte Groesse        : {self.letzte_map}")
        zeilen.append(f"  Laenge != w*h         : {self.map_fehl['len']}")
        zeilen.append(f"  Aufloesung ungueltig  : {self.map_fehl['res']}")
        zeilen.append(f"  Quaternion unnormiert : {self.map_fehl['quat']}")
        zeilen.append(f"  groesste Abw. ||q|-1| : {self.quat_abw_max:.3e}")
        zeilen.append(f"  Nav2 hat verworfen    : {self.zaehler['map_malformed']}")
        if self.zaehler["map_malformed"] and not sum(self.map_fehl.values()):
            zeilen.append("  >> Nav2 verwirft, meine Pruefung nicht -> Nav2 prueft mehr,"
                          " oder es ist der Transport (eigener Teilnehmer).")

        zeilen.append("\n--- Distanz-Gegenprobe ---")
        zeilen.append(f"  /odom integriert      : {self.d_odom:.2f} m")
        if self.eval_csv:
            f = self.eval_csv.split(";")
            if len(f) >= 7:
                try:
                    zeilen.append(f"  Explorer d_ges        : {float(f[6]):.2f} m")
                    zeilen.append(f"  Differenz             : "
                                  f"{abs(self.d_odom - float(f[6])):.2f} m")
                except ValueError:
                    pass

        zeilen.append("\n--- Kollisionsmonitor ---")
        zeilen.append(f"  Eingriffe             : {self.cm_events}"
                      f"{'' if HAT_CM else '   (Typ nicht importierbar, nicht gemessen)'}")

        zeilen.append("\n--- [EVAL] des Explorers ---")
        zeilen.append(f"  {'CSV;' + self.eval_csv if self.eval_csv else 'NICHT ERFASST'}")
        if not self.eval_csv:
            zeilen.append("  >> Lauf unvollstaendig oder zu frueh beendet.")
        zeilen.append("=" * 68)

        text = "\n".join(zeilen)
        print("\n" + text, flush=True)

        with open(os.path.join(o.outdir, f"{o.name}_zusammenfassung.txt"), "w",
                  encoding="utf-8") as fh:
            fh.write(text + "\n")

        # Ereignis-CSV: ein Abschnitt pro Zeile -> Datensatz fuer Kapitel 5
        with open(os.path.join(o.outdir, f"{o.name}_abschnitte.csv"), "w",
                  newline="", encoding="utf-8") as fh:
            wr = csv.writer(fh, delimiter=";")
            wr.writerow(["lauf", "nr", "start_x", "start_y", "ziel_x", "ziel_y",
                         "dist_m", "typ", "achse_deg", "planer_fehler"])
            for i, a in enumerate(self.abschnitte, 1):
                wr.writerow([o.name, i, f"{a['start'][0]:.2f}", f"{a['start'][1]:.2f}",
                             f"{a['ziel'][0]:.2f}", f"{a['ziel'][1]:.2f}",
                             f"{a['dist']:.2f}", a["typ"], f"{a['achse']:.2f}",
                             a["fehler"]])

        self.schreibe_gesamt(ger_f, eck_f)

    def schreibe_gesamt(self, ger_f, eck_f):
        o = self.opt
        e = (self.eval_csv.split(";") if self.eval_csv else [""] * 13)
        e += [""] * (13 - len(e))
        clearance = e[12]
        gruppe = "neu" if clearance not in ("", "0", "0.0") else \
                 ("base" if clearance in ("0", "0.0") else "?")

        zeile = {
            "name": o.name, "szene": o.szene, "gruppe": gruppe,
            "ziele": e[0], "t_expl": e[1], "t_rth": e[2], "t_ges": e[3],
            "d_expl": e[4], "d_rth": e[5], "d_ges": e[6], "rth": e[7],
            "blacklist": e[8], "reval": e[9], "retry": e[10], "timeout": e[11],
            "clearance": clearance,
            "d_odom": f"{self.d_odom:.2f}",
            "planer_fehler": self.zaehler["planer_fehler"],
            "abschnitte": len(self.abschnitte),
            "gerade_fehler": ger_f, "ecke_fehler": eck_f,
            "spin": self.zaehler["spin"], "wait": self.zaehler["wait"],
            "backup": self.zaehler["backup"],
            "costmap_clear": self.zaehler["costmap_clear"],
            "map_malformed_nav2": self.zaehler["map_malformed"],
            "map_uebersprungen_expl": self.zaehler["map_uebersprungen"],
            "map_n": self.map_n,
            "map_fehl_len": self.map_fehl["len"],
            "map_fehl_res": self.map_fehl["res"],
            "map_fehl_quat": self.map_fehl["quat"],
            "quat_abw_max": f"{self.quat_abw_max:.3e}",
            "collision_events": self.cm_events if HAT_CM else "n/a",
            "clearance_verworfen_max": self.clearance_max,
            "planer_hz_min": "" if self.planer_hz_min is None else f"{self.planer_hz_min:.4f}",
        }
        pfad = os.path.join(o.outdir, "eval_gesamt.csv")
        neu = not os.path.exists(pfad)
        with open(pfad, "a", newline="", encoding="utf-8") as fh:
            wr = csv.DictWriter(fh, fieldnames=GESAMT_SPALTEN, delimiter=";")
            if neu:
                wr.writeheader()
            wr.writerow(zeile)
        print(f"\n-> angehaengt an {pfad}", flush=True)
        print(f"-> {o.name}_explorer.log, _zusammenfassung.txt, _abschnitte.csv "
              f"in {o.outdir}", flush=True)


def name_schon_vergeben(outdir, name):
    """Namensdisziplin erzwingen (Falle 7): denselben Namen nicht zweimal."""
    for datei, spalte in (("probe_ergebnisse.csv", 0), ("eval_gesamt.csv", 0)):
        pfad = os.path.join(outdir, datei)
        if not os.path.exists(pfad):
            continue
        with open(pfad, encoding="utf-8") as fh:
            for zeile in fh:
                if zeile.split(";")[spalte].strip() == name:
                    return datei
    return None


def main():
    argv = remove_ros_args(args=sys.argv)
    ap = argparse.ArgumentParser(description="eval_probe2 -- Ergaenzung zu eval_probe.py")
    ap.add_argument("name", help="Laufname, z.B. uni_neu_1")
    ap.add_argument("--szene", default="uni", choices=["uni", "sr", "keine"],
                    help="uni = Uni-Ringflur (Abschnitte werden klassifiziert)")
    ap.add_argument("--outdir", default="/root/maps/eval")
    ap.add_argument("--force", action="store_true", help="Namenspruefung uebergehen")
    opt = ap.parse_args(argv[1:])

    os.makedirs(opt.outdir, exist_ok=True)
    doppelt = name_schon_vergeben(opt.outdir, opt.name)
    if doppelt and not opt.force:
        print(f"ABBRUCH: '{opt.name}' steht schon in {doppelt}.\n"
              f"         Anderen Namen waehlen oder --force setzen.", file=sys.stderr)
        sys.exit(1)

    rclpy.init(args=sys.argv)
    knoten = EvalProbe2(opt)
    try:
        rclpy.spin(knoten)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            knoten.abschluss()
        finally:
            knoten.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()

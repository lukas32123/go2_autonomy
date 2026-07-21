#!/usr/bin/env python3
"""
patch_saeulenraum.py -- ersetzt design_scene() in play_go2_ros_scan.py durch den
"Saeulenraum": 20 x 20 m Halle mit unregelmaessig gestellten quadratischen
Saeulen. Szenario "stark zugestellter Innenraum".

Warum dieses Szenario:
  Im leeren Flur liegen Frontier-Ziele fast immer mittig, die geometrische
  Zielpruefung (goal_clearance) greift daher kaum -- messbar daran, dass die
  base-Gruppe im Ringflur zwar 30 % langsamer war, aber NULL Blacklist- und
  Timeout-Ereignisse hatte. Zwischen Saeulen liegt dagegen ein grosser Teil der
  Frontiers in Luecken, die der Roboter nicht einnehmen kann. Erst hier wird
  sichtbar, wofuer Zielpruefung, Retry und Blacklist gebaut wurden.

Unregelmaessig, nicht im Raster:
  Ein exaktes Raster ist auf andere Weise degeneriert -- der Scan-Matcher kann
  auf die falsche Saeule einrasten (Aliasing). Die Saeulen werden deshalb aus
  einem Grundraster deterministisch ausgelenkt. Deterministisch heisst: die
  Szene ist bit-genau reproduzierbar, kein Zufallsgenerator, kein Seed.

SICHERHEIT -- schreibt NICHT ohne ausdrueckliche Freigabe:
    python3 patch_saeulenraum.py            # zeigt nur, was passieren wuerde
    python3 patch_saeulenraum.py --apply    # schreibt wirklich
  So kann das Skript nicht versehentlich eine laufende Messreihe zerstoeren.

Sicherung & Rueckweg:
  Beim ersten --apply wird play_go2_ros_scan.py.bak_vor_saeulen angelegt (eine
  vorhandene Sicherung wird NICHT ueberschrieben). Zurueck zum Uni-Flur:
      cp play_go2_ros_scan.py.bak_vor_saeulen play_go2_ros_scan.py
  Danach Isaac neu starten. Wirksam wird der Patch erst beim naechsten Start.

Verankert an 'def design_scene' + 'return Articulation(robot_cfg)'. Passen die
Anker nicht exakt, bricht das Skript ab OHNE zu schreiben. Nur Standardbibliothek.

Alle Masse stehen als Konstanten am Anfang von design_scene() -- Saeulengroesse,
Raster und Auslenkung lassen sich dort aendern, ohne ein neues Skript zu schreiben.
"""
import pathlib
import sys

TARGET = pathlib.Path(__file__).resolve().parent / "play_go2_ros_scan.py"
BACKUP = pathlib.Path(str(TARGET) + ".bak_vor_saeulen")

START = "def design_scene() -> Articulation:"
END = "    return Articulation(robot_cfg)"

NEW_FUNC = '''def design_scene() -> Articulation:
    import math

    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/ground", ground_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=750.0, color=(1.0, 1.0, 1.0))
    light_cfg.func("/World/Light", light_cfg)

    # ---------------- Saeulenraum (parametrisch) ----------------
    ROOM        = 20.0    # Aussenmass, Wand-Mittellinie [m]
    WALL_T      = 0.20    # Wanddicke [m]
    WALL_H      = 1.00    # Wandhoehe [m]
    PILLAR      = 0.60    # Kantenlaenge der quadratischen Saeulen [m]
    PITCH       = 2.50    # Grundraster [m]
    JITTER      = 0.35    # max. Auslenkung aus dem Raster [m]  (0.0 = exaktes Raster)
    EDGE_MARGIN = 1.60    # Mindestabstand Saeulenmitte <-> Wandflaeche [m]
    START_CLEAR = 1.80    # Radius um die Startpose ohne Saeule [m]

    half = ROOM / 2.0
    inner = half - WALL_T / 2.0        # Innenflaeche der Waende
    z = WALL_H / 2.0

    gray = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.60, 0.60, 0.60))
    dark = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.42, 0.42, 0.48))
    coll = sim_utils.CollisionPropertiesCfg()

    # --- Aussenwaende ---
    # WICHTIG: USD erlaubt in Prim-Namen KEIN Minuszeichen. Deshalb wird ueber
    # den Index (0/1) benannt, nicht ueber das Vorzeichen.
    for idx, s in enumerate((1.0, -1.0)):
        w = sim_utils.CuboidCfg(size=(ROOM + WALL_T, WALL_T, WALL_H),
                                visual_material=gray, collision_props=coll)
        w.func("/World/wall_ns_%d" % idx, w, translation=(0.0, s * half, z))
        w = sim_utils.CuboidCfg(size=(WALL_T, ROOM + WALL_T, WALL_H),
                                visual_material=gray, collision_props=coll)
        w.func("/World/wall_ew_%d" % idx, w, translation=(s * half, 0.0, z))

    # --- Startpose: freie Bucht an der Suedwand ---
    start_x, start_y = 0.0, -(inner - 1.20)

    def jitter(i, j, salt):
        """Deterministische Auslenkung in [-1, 1). Kein Zufallsgenerator."""
        v = math.sin(i * 127.1 + j * 311.7 + salt * 74.7) * 43758.5453
        return (v - math.floor(v)) * 2.0 - 1.0

    lim = inner - EDGE_MARGIN - PILLAR / 2.0 - JITTER
    ks = []
    k = 0
    while k * PITCH <= lim:
        ks.extend([0] if k == 0 else [-k, k])
        k += 1
    ks.sort()

    n = 0
    skipped = 0
    for i in ks:
        for j in ks:
            px = i * PITCH + JITTER * jitter(i, j, 1.0)
            py = j * PITCH + JITTER * jitter(i, j, 2.0)
            if math.hypot(px - start_x, py - start_y) < START_CLEAR:
                skipped += 1
                continue
            p = sim_utils.CuboidCfg(size=(PILLAR, PILLAR, WALL_H),
                                    visual_material=dark, collision_props=coll)
            p.func("/World/pillar_%d" % n, p, translation=(px, py, z))
            n += 1

    print("[SZENE] Saeulenraum %.1f x %.1f m | %d Saeulen a %.2f m "
          "(Raster %.2f m, Auslenkung +-%.2f m, %d an der Startpose ausgelassen)"
          % (ROOM, ROOM, n, PILLAR, PITCH, JITTER, skipped))

    robot_cfg = UNITREE_GO2_CFG.replace(prim_path="/World/Robot")
    robot_cfg.init_state.pos = (start_x, start_y, 0.4)
    return Articulation(robot_cfg)
'''


def main():
    apply = "--apply" in sys.argv[1:]

    if not TARGET.exists():
        sys.exit("FEHLER: %s nicht gefunden." % TARGET)
    src = TARGET.read_text()

    n = src.count(START)
    if n != 1:
        sys.exit("FEHLER: '%s' %dx gefunden (erwartet genau 1). "
                 "Abbruch, nichts geaendert." % (START, n))
    i = src.index(START)
    j = src.find(END, i)
    if j == -1:
        sys.exit("FEHLER: '%s' nach der Funktion nicht gefunden. "
                 "Abbruch, nichts geaendert." % END.strip())
    j_end = j + len(END)

    if not apply:
        print("PROBELAUF -- es wurde NICHTS geaendert.")
        print("  Ziel     : %s" % TARGET)
        print("  Anker    : gefunden, Zeilen %d bis %d"
              % (src[:i].count("\\n") + 1, src[:j_end].count("\\n") + 1))
        print("  Sicherung: %s (%s)"
              % (BACKUP.name, "existiert bereits" if BACKUP.exists() else "wird angelegt"))
        print("  Ersetzt  : design_scene() -> Saeulenraum 20 x 20 m")
        print("\\nZum wirklichen Anwenden:  python3 %s --apply" % pathlib.Path(__file__).name)
        print("ACHTUNG: erst nach Abschluss der laufenden Messreihe anwenden.")
        return

    if not BACKUP.exists():
        BACKUP.write_text(src)
        print("Sicherung angelegt: %s" % BACKUP)
    else:
        print("Sicherung existiert bereits (nicht ueberschrieben): %s" % BACKUP)

    TARGET.write_text(src[:i] + NEW_FUNC.rstrip("\\n") + src[j_end:])
    print("design_scene() ersetzt -> Saeulenraum.")
    print("Naechster Schritt: Isaac neu starten:")
    print("   %s/run_go2_scan.sh --laser_z 0.40" % TARGET.parent)
    print("Zurueck zum Uni-Flur:  cp %s %s" % (BACKUP.name, TARGET.name))


if __name__ == "__main__":
    main()

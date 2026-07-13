#!/usr/bin/env python3
"""
patch_ringflur.py — ersetzt design_scene() in play_go2_ros_scan.py durch die
Ring-Flur-Testumgebung (quadratischer Aussenring + solider Innenblock + Flur).

Sicher & idempotent:
  - Legt beim ersten Lauf eine Sicherung an (…play_go2_ros_scan.py.bak_smallroom);
    ueberschreibt eine vorhandene Sicherung NICHT.
  - Ersetzt die gesamte Funktion, verankert an 'def design_scene' + 'return
    Articulation(robot_cfg)'. Passen die Anker nicht exakt (Datei anders als
    erwartet), bricht das Skript ab, OHNE etwas zu schreiben.
  - Nutzt nur die Standardbibliothek. Aufruf: python3 patch_ringflur.py

Rueckweg: cp …bak_smallroom …play_go2_ros_scan.py  (dann Isaac neu starten).
"""
import pathlib
import sys

TARGET = pathlib.Path(__file__).resolve().parent / "play_go2_ros_scan.py"
BACKUP = pathlib.Path(str(TARGET) + ".bak_smallroom")

START = "def design_scene() -> Articulation:"
END = "    return Articulation(robot_cfg)"

NEW_FUNC = '''def design_scene() -> Articulation:
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/ground", ground_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=750.0, color=(1.0, 1.0, 1.0))
    light_cfg.func("/World/Light", light_cfg)

    # --- Ring-Flur-Testumgebung (parametrisch) ------------------------------
    # Quadratischer Aussenring (4 Waende) um einen soliden Innenblock; dazwischen
    # der Flur. Der LiDAR sieht an jeder Ecke nicht um die Kurve -> echte Frontiers,
    # die Schleife muss komplett abgelaufen werden (lange Strecke fuer Return-to-Home).
    OUTER = 10.0        # Aussenmass (Wand-Mittellinie), m
    CORRIDOR = 2.0      # Flurbreite, m  (falls Nav2 zu eng plant: auf 2.5 erhoehen)
    WALL_T = 0.1        # Wanddicke, m
    WALL_H = 1.0        # Wandhoehe, m (LiDAR auf 0.12 m -> klar sichtbar)
    half = OUTER / 2.0
    inner = OUTER - 2.0 * CORRIDOR      # Innenblock-Kantenlaenge
    z = WALL_H / 2.0                    # Zentrum-z, Wand steht auf dem Boden
    gray = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.6, 0.6, 0.6))
    dark = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.45, 0.45, 0.45))
    coll = sim_utils.CollisionPropertiesCfg()

    # 4 Aussenwaende (Nord/Sued lang in x, Ost/West lang in y)
    wall_ns = sim_utils.CuboidCfg(size=(OUTER, WALL_T, WALL_H), visual_material=gray, collision_props=coll)
    wall_ns.func("/World/wall_n", wall_ns, translation=(0.0, half, z))
    wall_ns.func("/World/wall_s", wall_ns, translation=(0.0, -half, z))
    wall_ew = sim_utils.CuboidCfg(size=(WALL_T, OUTER, WALL_H), visual_material=gray, collision_props=coll)
    wall_ew.func("/World/wall_e", wall_ew, translation=(half, 0.0, z))
    wall_ew.func("/World/wall_w", wall_ew, translation=(-half, 0.0, z))

    # Solider Innenblock -> Mitte unpassierbar, erzwingt die Schleife
    block = sim_utils.CuboidCfg(size=(inner, inner, WALL_H), visual_material=dark, collision_props=coll)
    block.func("/World/inner_block", block, translation=(0.0, 0.0, z))
    # ------------------------------------------------------------------------

    robot_cfg = UNITREE_GO2_CFG.replace(prim_path="/World/Robot")
    # Startpose IM SUED-FLUR (Ursprung ist jetzt der Block!). Blick +x (Ost), umrundet den Block.
    start_y = -(inner / 2.0 + CORRIDOR / 2.0)
    robot_cfg.init_state.pos = (0.0, start_y, 0.4)
    return Articulation(robot_cfg)
'''


def main():
    if not TARGET.exists():
        sys.exit(f"FEHLER: {TARGET} nicht gefunden.")
    src = TARGET.read_text()

    n = src.count(START)
    if n != 1:
        sys.exit(f"FEHLER: '{START}' {n}x gefunden (erwartet genau 1). "
                 f"Abbruch, nichts geaendert.")
    i = src.index(START)
    j = src.find(END, i)
    if j == -1:
        sys.exit(f"FEHLER: '{END.strip()}' nach der Funktion nicht gefunden. "
                 f"Abbruch, nichts geaendert.")
    j_end = j + len(END)

    if not BACKUP.exists():
        BACKUP.write_text(src)
        print(f"Sicherung angelegt: {BACKUP}")
    else:
        print(f"Sicherung existiert bereits (nicht ueberschrieben): {BACKUP}")

    new_src = src[:i] + NEW_FUNC.rstrip("\n") + src[j_end:]
    TARGET.write_text(new_src)

    a = new_src.index(START)
    b = new_src.find(END, a) + len(END)
    print("design_scene() ersetzt. Neue Funktion zur Kontrolle:")
    print("-" * 66)
    print(new_src[a:b])
    print("-" * 66)
    print(f"OK. Naechster Schritt: Isaac neu starten ({TARGET.parent}/run_go2_scan.sh).")


if __name__ == "__main__":
    main()

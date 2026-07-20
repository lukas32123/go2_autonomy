#!/usr/bin/env python3
"""
patch_unigang.py -- ersetzt design_scene() in play_go2_ros_scan.py durch den
"Uni-Ringflur": 30 m Aussenmass, zwei gegenueberliegende breite Fluren (6 m),
zwei schmale (3 m), mit Tuernischen als Merkmale fuer das Scan-Matching.

Warum: Der glatte Ringflur ist geometrisch degeneriert -- entlang eines Flurs
sieht jede Position aus wie jede andere, das Scan-Matching hat nichts zum
Festhalten (gemessen: map->odom-Yaw wandert 4.6-7.6 deg trotz exakter
Odometrie). Nischen geben dem Matcher Ecken. Der Grundriss ist einem realen
Uni-Gebaeude nachempfunden.

Bauweise (spart Prims und erzeugt exakte Rechtecknischen):
  Statt Nischen aus der Wand zu "schneiden", steht die Rueckwand durchgehend
  aussen, und PFEILER springen nach innen vor. Die Luecken zwischen den
  Pfeilern SIND die Nischen. 41 Prims statt ~140.

Nischen laut Vorgabe:
  - breite Seiten (Nord/Sued, 6 m Flur): alle 6 m, NUR aussen
  - schmale Seiten (Ost/West, 3 m Flur): alle 4 m, aussen UND am Kern

Sicher & idempotent (wie patch_ringflur.py):
  - Sicherung …play_go2_ros_scan.py.bak_ringflur beim ersten Lauf; eine
    vorhandene Sicherung wird NICHT ueberschrieben.
  - Verankert an 'def design_scene' + 'return Articulation(robot_cfg)'.
    Passen die Anker nicht exakt, bricht das Skript ab OHNE zu schreiben.
  - Nur Standardbibliothek. Aufruf: python3 patch_unigang.py

Rueckweg: cp …bak_ringflur …play_go2_ros_scan.py  (dann Isaac neu starten).

Alle Masse sind Konstanten am Anfang von design_scene() -- Nischentiefe,
-breite, Abstaende und Flurbreiten lassen sich dort aendern, ohne ein neues
Patch-Skript zu schreiben.
"""
import pathlib
import sys

TARGET = pathlib.Path(__file__).resolve().parent / "play_go2_ros_scan.py"
BACKUP = pathlib.Path(str(TARGET) + ".bak_ringflur")

START = "def design_scene() -> Articulation:"
END = "    return Articulation(robot_cfg)"

NEW_FUNC = '''def design_scene() -> Articulation:
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/ground", ground_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=750.0, color=(1.0, 1.0, 1.0))
    light_cfg.func("/World/Light", light_cfg)

    # ---------------- Uni-Ringflur (parametrisch) ----------------
    # Grundriss nach realem Uni-Gebaeude: Ring um einen soliden Kern, zwei
    # gegenueberliegende Fluren breit, zwei schmal, Tuernischen als Merkmale.
    OUTER         = 30.0    # Aussenmass, Mittellinie der Nischen-Rueckwand [m]
    CORR_WIDE     = 6.0     # Flurbreite Nord/Sued [m]
    CORR_NARROW   = 3.0     # Flurbreite Ost/West [m]  (Vorgabe 2.5-4.0)
    NICHE_DEPTH   = 0.40    # Nischentiefe [m]
    NICHE_WIDTH   = 1.40    # Nischenbreite (Tuerbreite + Rahmen) [m]
    PITCH_WIDE    = 6.0     # Nischenabstand auf den breiten Seiten [m]
    PITCH_NARROW  = 4.0     # Nischenabstand auf den schmalen Seiten [m]
    CORNER_MARGIN = 1.5     # keine Nische naeher als das an einer Ecke [m]
    WALL_T        = 0.20    # Wanddicke [m]
    WALL_H        = 1.00    # Wandhoehe [m]

    half = OUTER / 2.0
    back = half - WALL_T / 2.0            # Innenflaeche der Rueckwand (Nischengrund)
    pier = back - NICHE_DEPTH             # Flurgrenze aussen (Pfeilerflaeche)
    core_y = pier - CORR_WIDE             # Kernflaeche Nord/Sued
    core_x = pier - CORR_NARROW           # Kernflaeche Ost/West (Pfeilerflaeche)
    core_solid = core_x - NICHE_DEPTH     # Kern-Vollkoerper reicht bis hier
    z = WALL_H / 2.0

    gray = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.60, 0.60, 0.60))
    dark = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.45, 0.45, 0.45))
    coll = sim_utils.CollisionPropertiesCfg()

    def niche_centers(length, pitch):
        """Nischenmitten, symmetrisch um 0, mit Abstand zu den Ecken."""
        lim = length / 2.0 - CORNER_MARGIN - NICHE_WIDTH / 2.0
        out, k = [], 0
        while k * pitch <= lim:
            out.extend([0.0] if k == 0 else [-k * pitch, k * pitch])
            k += 1
        return sorted(out)

    def pier_spans(length, centers):
        """Intervalle zwischen den Nischen -> dort stehen die Pfeiler."""
        out, lo = [], -length / 2.0
        for c in centers:
            hi = c - NICHE_WIDTH / 2.0
            if hi - lo > 0.05:
                out.append((lo, hi))
            lo = c + NICHE_WIDTH / 2.0
        if length / 2.0 - lo > 0.05:
            out.append((lo, length / 2.0))
        return out

    n = 0
    for s in (1.0, -1.0):
        # --- breite Seiten (Nord/Sued): Rueckwand + Pfeiler, Nischen nur aussen
        w = sim_utils.CuboidCfg(size=(OUTER, WALL_T, WALL_H),
                                visual_material=gray, collision_props=coll)
        w.func("/World/wall_ns_%d" % n, w, translation=(0.0, s * half, z))
        n += 1
        for a, b in pier_spans(OUTER, niche_centers(OUTER, PITCH_WIDE)):
            p = sim_utils.CuboidCfg(size=(b - a, NICHE_DEPTH, WALL_H),
                                    visual_material=gray, collision_props=coll)
            p.func("/World/pier_ns_%d" % n, p,
                   translation=((a + b) / 2.0, s * (pier + NICHE_DEPTH / 2.0), z))
            n += 1
        # --- schmale Seiten (Ost/West): Rueckwand + Pfeiler
        w = sim_utils.CuboidCfg(size=(WALL_T, OUTER, WALL_H),
                                visual_material=gray, collision_props=coll)
        w.func("/World/wall_ew_%d" % n, w, translation=(s * half, 0.0, z))
        n += 1
        for a, b in pier_spans(OUTER, niche_centers(OUTER, PITCH_NARROW)):
            p = sim_utils.CuboidCfg(size=(NICHE_DEPTH, b - a, WALL_H),
                                    visual_material=gray, collision_props=coll)
            p.func("/World/pier_ew_%d" % n, p,
                   translation=(s * (pier + NICHE_DEPTH / 2.0), (a + b) / 2.0, z))
            n += 1

    # --- Kern: Vollkoerper, dazu Pfeiler auf den Ost/West-Flaechen ---------
    core = sim_utils.CuboidCfg(size=(2.0 * core_solid, 2.0 * core_y, WALL_H),
                               visual_material=dark, collision_props=coll)
    core.func("/World/core", core, translation=(0.0, 0.0, z))
    n += 1
    core_len = 2.0 * core_y
    for s in (1.0, -1.0):
        for a, b in pier_spans(core_len, niche_centers(core_len, PITCH_NARROW)):
            p = sim_utils.CuboidCfg(size=(NICHE_DEPTH, b - a, WALL_H),
                                    visual_material=dark, collision_props=coll)
            p.func("/World/core_pier_%d" % n, p,
                   translation=(s * (core_solid + NICHE_DEPTH / 2.0), (a + b) / 2.0, z))
            n += 1

    print("[SZENE] Uni-Ringflur %.1f m | Flur N/S %.1f m, O/W %.1f m | "
          "Nische %.2f x %.2f m | %d Prims"
          % (OUTER, pier - core_y, pier - core_x, NICHE_WIDTH, NICHE_DEPTH, n))

    robot_cfg = UNITREE_GO2_CFG.replace(prim_path="/World/Robot")
    # Startpose in der Mitte des SUEDflurs (breite Seite), Blick +x (Ost).
    start_y = -(core_y + CORR_WIDE / 2.0)
    robot_cfg.init_state.pos = (0.0, start_y, 0.4)
    return Articulation(robot_cfg)
'''


def main():
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

    if not BACKUP.exists():
        BACKUP.write_text(src)
        print("Sicherung angelegt: %s" % BACKUP)
    else:
        print("Sicherung existiert bereits (nicht ueberschrieben): %s" % BACKUP)

    new_src = src[:i] + NEW_FUNC.rstrip("\n") + src[j_end:]
    TARGET.write_text(new_src)

    a = new_src.index(START)
    b = new_src.find(END, a) + len(END)
    print("design_scene() ersetzt. Kontrolle (erste 25 Zeilen):")
    print("-" * 66)
    print("\n".join(new_src[a:b].splitlines()[:25]))
    print("   ...")
    print("-" * 66)
    print("OK. Naechster Schritt: Isaac neu starten:")
    print("   %s/run_go2_scan.sh --laser_z 0.40" % TARGET.parent)


if __name__ == "__main__":
    main()

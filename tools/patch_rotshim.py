#!/usr/bin/env python3
"""
patch_rotshim.py — umhuellt den MPPI-Controller in nav2_go2.yaml mit dem
RotationShimController (behebt den Gross-Drehungs-Haenger). Backup + idempotent.

Aendert genau die 'plugin:'-Zeile des FollowPath-Controllers und fuegt die
Shim-Parameter ein. Alle MPPI-Parameter + Critics bleiben unveraendert (der Shim
reicht das Fahren an MPPI als primary_controller weiter). Bricht sicher ab (ohne
Schreiben), falls die erwartete Zeile nicht genau einmal vorkommt.

Rueckweg: cp …bak_prerotshim …nav2_go2.yaml  (dann Nav2 neu starten).
Aufruf:   python3 patch_rotshim.py
"""
import pathlib
import sys

TARGET = pathlib.Path.home() / "welt2_slam" / "maps" / "nav2_go2.yaml"
BACKUP = pathlib.Path(str(TARGET) + ".bak_prerotshim")

OLD = 'plugin: "nav2_mppi_controller::MPPIController"'
NEW = (
    'plugin: "nav2_rotation_shim_controller::RotationShimController"\n'
    '      primary_controller: "nav2_mppi_controller::MPPIController"\n'
    '      # RotationShimController: dreht bei grossem Anfangs-Winkelfehler zuerst\n'
    '      # auf der Stelle (rein wz, wie Handbetrieb), uebergibt dann an MPPI.\n'
    '      angular_dist_threshold: 0.785        # rad (~45 deg): ab hier erst eindrehen\n'
    '      forward_sampling_distance: 0.5       # m: Pfadpunkt voraus zur Zielrichtung\n'
    '      rotate_to_heading_angular_vel: 1.2   # rad/s Drehrate (konservativ, tunebar)\n'
    '      max_angular_accel: 3.2               # rad/s^2\n'
    '      simulate_ahead_time: 1.0             # s: Kollisionscheck waehrend Drehung'
)


def main():
    if not TARGET.exists():
        sys.exit(f"FEHLER: {TARGET} nicht gefunden.")
    src = TARGET.read_text()

    if 'RotationShimController' in src:
        print("Hinweis: RotationShimController bereits vorhanden — nichts zu tun.")
        return

    n = src.count(OLD)
    if n != 1:
        sys.exit(f"FEHLER: erwartete Zeile {n}x gefunden (erwartet genau 1). "
                 f"Abbruch, nichts geaendert.")

    if not BACKUP.exists():
        BACKUP.write_text(src)
        print(f"Sicherung angelegt: {BACKUP}")
    else:
        print(f"Sicherung existiert bereits (nicht ueberschrieben): {BACKUP}")

    TARGET.write_text(src.replace(OLD, NEW, 1))
    print("FollowPath auf RotationShimController umgestellt. Kontrolle:")
    print("-" * 66)
    out = TARGET.read_text().splitlines()
    for i, line in enumerate(out):
        if "FollowPath:" in line:
            for l in out[i:i + 12]:
                print(l)
            break
    print("-" * 66)
    print("OK. Naechster Schritt: Nav2 (Terminal 3) neu starten.")


if __name__ == "__main__":
    main()

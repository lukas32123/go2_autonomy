#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
anhang_b_konfiguration.py

Zieht die Konfigurationsdateien des Autonomie-Stacks aus dem Repository und
erzeugt daraus den LaTeX-Block fuer Anhang B.

Aufruf aus ~/Schreiben/abb heraus:

    source venv/bin/activate
    python3 anhang_b_konfiguration.py --out pictures

Vorgabe fuer --repo ist ~/go2_autonomy. Geschrieben werden
    <out>/code/*.yaml, *.xml, Dockerfile.*   die abzudruckenden Auszuege
    <out>/anhang_b_konfiguration.tex         der LaTeX-Block

Die Auszuege werden bei jedem Lauf neu aus dem Repository gezogen. Damit ist
ausgeschlossen, dass im Anhang eine Fassung steht, die von der tatsaechlich
verwendeten abweicht.

Abhaengigkeiten: keine, nur die Standardbibliothek.
"""

import argparse
import io
import os
import subprocess

MARKE = "# [...] hier ausgelassen, vollstaendig im Repository"


# --------------------------------------------------------------------------
def lies(pfad):
    if not os.path.isfile(pfad):
        raise SystemExit(f"Datei nicht gefunden: {pfad}")
    return io.open(pfad, encoding="utf-8").read().splitlines()


def yaml_block(zeilen, schluessel):
    """Schneidet einen Abschnitt oberster Ebene aus einer YAML-Datei.

    Gesucht wird die Zeile, die ohne Einrueckung mit dem Schluessel beginnt.
    Der Abschnitt endet vor der naechsten Zeile ohne Einrueckung.
    """
    start = None
    for i, z in enumerate(zeilen):
        if z.startswith(schluessel + ":"):
            start = i
            break
    if start is None:
        raise SystemExit(f"Abschnitt {schluessel} nicht gefunden")
    ende = len(zeilen)
    for i in range(start + 1, len(zeilen)):
        z = zeilen[i]
        if z.strip() and not z[0].isspace():
            ende = i
            break
    while ende > start and not zeilen[ende - 1].strip():
        ende -= 1
    return zeilen[start:ende]


def kuerze_nach(block, marker, marke=MARKE):
    """Behaelt den Block bis einschliesslich der Zeile, die marker enthaelt,
    und ersetzt den Rest durch eine Auslassungsmarke."""
    for i, z in enumerate(block):
        if marker in z:
            einzug = " " * (len(z) - len(z.lstrip()))
            return block[:i + 1] + [einzug + marke]
    return block


def schreibe(pfad, zeilen):
    io.open(pfad, "w", encoding="utf-8").write("\n".join(zeilen) + "\n")
    print(f"  geschrieben: {pfad}  ({len(zeilen)} Zeilen)")
    return len(zeilen)


# --------------------------------------------------------------------------
def stand_des_repos(repo):
    """Liest Commit und Tag des Repositorys, damit der Anhang den Stand nennt."""
    def git(*args):
        try:
            return subprocess.run(["git", "-C", repo] + list(args),
                                  capture_output=True, text=True,
                                  check=True).stdout.strip()
        except Exception:
            return ""
    return git("rev-parse", "--short", "HEAD"), git("describe", "--tags", "--always")


# --------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser()
    p.add_argument("--repo", default="~/go2_autonomy",
                   help="Wurzel des Repositorys go2_autonomy")
    p.add_argument("--out", default="pictures",
                   help="Ausgabeordner, darunter wird code/ angelegt")
    p.add_argument("--codepfad", default="code",
                   help="Pfad, den lstinputlisting in der erzeugten .tex "
                        "voranstellt, relativ zum Ordner der main.tex")
    a = p.parse_args()

    repo = os.path.expanduser(a.repo)
    out = os.path.expanduser(a.out)
    if not os.path.isdir(repo):
        raise SystemExit(f"Repository nicht gefunden: {repo}\n"
                         f"    Pfad mit --repo angeben.")
    code = os.path.join(out, "code")
    os.makedirs(code, exist_ok=True)

    commit, tag = stand_des_repos(repo)
    print(f"Repository {repo}")
    print(f"Stand      {tag or commit or 'unbekannt'}")
    print()

    nav = lies(os.path.join(repo, "nav2_slam", "nav2_go2.yaml"))
    umfang = {}

    # ---- vollstaendige Dateien --------------------------------------------
    print("Vollstaendige Dateien")
    for quelle, ziel in [
        (("nav2_slam", "mapper_params_go2.yaml"), "mapper_params_go2.yaml"),
        (("nav2_slam", "fastdds_udp.xml"),        "fastdds_udp.xml"),
        (("nav2_slam", "Dockerfile.slam"),        "Dockerfile.slam"),
        (("nav2_slam", "Dockerfile.nav"),         "Dockerfile.nav"),
    ]:
        umfang[ziel] = schreibe(os.path.join(code, ziel),
                                lies(os.path.join(repo, *quelle)))

    # ---- Auszuege aus nav2_go2.yaml ---------------------------------------
    print("\nAuszuege aus nav2_go2.yaml")
    regler = kuerze_nach(yaml_block(nav, "controller_server"),
                         '"PathAngleCritic", "PreferForwardCritic"]')
    auszuege = [
        ("nav2_planer.yaml",            yaml_block(nav, "planner_server")),
        ("nav2_regler.yaml",            regler),
        ("nav2_kostenkarte_lokal.yaml", yaml_block(nav, "local_costmap")),
        ("nav2_kostenkarte_global.yaml", yaml_block(nav, "global_costmap")),
        ("nav2_glaettung.yaml",         yaml_block(nav, "velocity_smoother")),
        ("nav2_kollisionsmonitor.yaml", yaml_block(nav, "collision_monitor")),
    ]
    for name, block in auszuege:
        umfang[name] = schreibe(os.path.join(code, name), block)

    # ---- LaTeX-Block -------------------------------------------------------
    tex = ["% Erzeugt von anhang_b_konfiguration.py, nicht von Hand bearbeiten.",
           f"% Repository-Stand {tag or commit}", ""]

    def listing(datei, unterschrift, label, sprache=None):
        opt = [f"caption={{{unterschrift}}}", f"label={{{label}}}"]
        if sprache:
            opt.insert(0, f"language={sprache}")
        tex.append("\\lstinputlisting[" + ", ".join(opt) + "]"
                   f"{{{a.codepfad}/{datei}}}")
        tex.append("")

    tex.append("\\section{Kartierung}")
    tex.append("")
    listing("mapper_params_go2.yaml",
            "Vollständige Konfiguration der SLAM Toolbox.",
            "lst:mapper")

    tex.append("\\section{Navigation}")
    tex.append("")
    listing("nav2_planer.yaml",
            "Globaler Planer.", "lst:planer")
    listing("nav2_regler.yaml",
            "Lokaler Regler mit vorgeschaltetem Drehbaustein. Die Gewichtung "
            "der Bewertungsterme ist ausgelassen.", "lst:regler")
    listing("nav2_kostenkarte_lokal.yaml",
            "Lokale Kostenkarte mit Grundfläche und Aufblähung.",
            "lst:kostenkarte_lokal")
    listing("nav2_kostenkarte_global.yaml",
            "Globale Kostenkarte mit Grundfläche und Aufblähung.",
            "lst:kostenkarte_global")
    listing("nav2_glaettung.yaml",
            "Glättungsglied vor der Ausgabe der Fahrbefehle.", "lst:glaettung")
    listing("nav2_kollisionsmonitor.yaml",
            "Kollisionsmonitor als letzte Instanz vor der Ausgabe.",
            "lst:kollisionsmonitor")

    tex.append("\\section{Datentransport und Container}")
    tex.append("")
    listing("fastdds_udp.xml",
            "Transportprofil der Middleware.", "lst:dds", sprache="XML")
    listing("Dockerfile.slam",
            "Abbild des Kartierungscontainers.", "lst:dockerslam")
    listing("Dockerfile.nav",
            "Abbild des Navigationscontainers, aufsetzend auf dem "
            "Kartierungsabbild.", "lst:dockernav")

    ziel = os.path.join(out, "anhang_b_konfiguration.tex")
    io.open(ziel, "w", encoding="utf-8").write("\n".join(tex) + "\n")

    print(f"\n  geschrieben: {ziel}")
    print(f"\n  Umfang insgesamt {sum(umfang.values())} Zeilen Quelltext, "
          f"das entspricht etwa {sum(umfang.values())/48:.0f} Seiten.")


if __name__ == "__main__":
    main()

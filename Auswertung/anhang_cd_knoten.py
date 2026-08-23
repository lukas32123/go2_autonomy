#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
anhang_cd_knoten.py

Erzeugt Anhang C mit den Auszuegen aus dem Explorationsknoten und Anhang D mit
der Werkzeugtabelle und dem Stand des Repositorys.

Aufruf aus ~/Schreiben/abb heraus:

    source venv/bin/activate
    python3 anhang_cd_knoten.py --repo ~/go2_autonomy --out pictures \
                                --codepfad abb/pictures/code

Geschrieben werden
    <out>/code/explorer_*.py          die drei Auszuege
    <out>/anhang_c_knoten.tex         der LaTeX-Block von Anhang C
    <out>/anhang_d_werkzeuge.tex      der LaTeX-Block von Anhang D

Die Auszuege werden ueber den Namen der Methode gesucht und nicht ueber
Zeilennummern. Eine spaetere Umstrukturierung des Knotens verschiebt die
Auszuege daher nicht stillschweigend, sie faellt als Fehlermeldung auf.

Abhaengigkeiten: keine, nur die Standardbibliothek.
"""

import argparse
import io
import os
import subprocess

MARKE = "        # [...] hier ausgelassen, vollstaendig im Repository"


# --------------------------------------------------------------------------
def lies(pfad):
    if not os.path.isfile(pfad):
        raise SystemExit(f"Datei nicht gefunden: {pfad}")
    return io.open(pfad, encoding="utf-8").read().splitlines()


def methode(zeilen, name):
    """Schneidet eine Methode aus einer Klasse heraus.

    Gesucht wird die Zeile mit 'def <name>(', der Auszug endet vor der naechsten
    Zeile, die auf derselben Einrueckung mit def oder class beginnt.
    """
    start = None
    for i, z in enumerate(zeilen):
        if z.lstrip().startswith(f"def {name}("):
            start = i
            break
    if start is None:
        raise SystemExit(f"Methode {name} nicht gefunden")
    einzug = len(zeilen[start]) - len(zeilen[start].lstrip())
    ende = len(zeilen)
    for i in range(start + 1, len(zeilen)):
        z = zeilen[i]
        if not z.strip():
            continue
        e = len(z) - len(z.lstrip())
        if e <= einzug and (z.lstrip().startswith("def ")
                            or z.lstrip().startswith("class ")
                            or e < einzug):
            ende = i
            break
    while ende > start and not zeilen[ende - 1].strip():
        ende -= 1
    return zeilen[start:ende]


def kuerze_nach(block, marker, marke=MARKE):
    for i, z in enumerate(block):
        if marker in z:
            return block[:i + 1] + ["", marke]
    raise SystemExit(f"Schnittmarke nicht gefunden: {marker}")


def schreibe(pfad, zeilen):
    io.open(pfad, "w", encoding="utf-8").write("\n".join(zeilen) + "\n")
    print(f"  geschrieben: {pfad}  ({len(zeilen)} Zeilen)")
    return len(zeilen)


def stand_des_repos(repo):
    def git(*args):
        try:
            return subprocess.run(["git", "-C", repo] + list(args),
                                  capture_output=True, text=True,
                                  check=True).stdout.strip()
        except Exception:
            return ""
    return (git("rev-parse", "--short", "HEAD"),
            git("describe", "--tags", "--exact-match"),
            git("log", "-1", "--format=%ad", "--date=short"))


def tex_escape(s):
    for a, b in [("\\", "\\textbackslash{}"), ("_", "\\_"), ("&", "\\&"),
                 ("%", "\\%"), ("#", "\\#")]:
        s = s.replace(a, b)
    return s


# --------------------------------------------------------------------------
# Anhang D, Werkzeuge. Die Beschreibungen folgen den Kopfkommentaren der
# Dateien im Repository.
# --------------------------------------------------------------------------
WERKZEUGE = [
    ("Simulation, host-nativ \\quad \\texttt{isaac/}", [
        ("play\\_go2\\_ros\\_scan.py",
         "Aufbau der Szene, Roboter mit gelernter Lauffortbewegung, "
         "2D-Laserscanner. Gibt Scan, Odometrie, Transformationsbaum und "
         "Simulationszeit aus."),
        ("run\\_go2\\_scan.sh",
         "Startskript der Simulationsseite mit der Umgebung des "
         "Simulationsbundles."),
        ("patch\\_saeulenraum.py",
         "Setzt die Testumgebung Säulenraum in die Szene ein."),
        ("patch\\_unigang.py",
         "Setzt die Testumgebung Ringflur in die Szene ein."),
    ]),
    ("Autonomie im Container \\quad \\texttt{frontier/}, \\texttt{nav2\\_slam/}", [
        ("frontier\\_explorer.py",
         "Der Explorationsknoten mit Detektion, Zielauswahl, "
         "Fehlerbehandlung und Rückkehr zum Ausgangspunkt. Auszüge in "
         "Anhang~\\ref{app:knoten}."),
        ("nav2\\_go2.yaml,\\newline mapper\\_params\\_go2.yaml",
         "Konfiguration von Navigation und Kartierung, abgedruckt in "
         "Anhang~\\ref{app:konfiguration}."),
        ("Dockerfile.slam,\\newline Dockerfile.nav",
         "Abbilder der beiden Container."),
    ]),
    ("Messung \\quad \\texttt{tools/}", [
        ("eval\\_probe.py",
         "Passiver Messknoten. Erfasst Roll- und Nickwinkel des Rumpfes, die "
         "Bewegung der Nachführung zwischen Kartenbezug und Odometriebezug "
         "sowie Stillstände, sichert die Karte und misst die beiden "
         "Wandwinkel. Schreibt \\texttt{probe\\_ergebnisse.csv}."),
        ("eval\\_probe2.py",
         "Passiver Messknoten. Zählt Planerfehlschläge, Wiederherstellungs"
         "manöver, Bereinigungen der Kostenkarte, verworfene Kartennachrichten "
         "und Eingriffe des Kollisionsmonitors, misst die Odometriedistanz als "
         "Gegenprobe und übernimmt die Kennwerte des Explorationsknotens. "
         "Schreibt \\texttt{eval\\_gesamt.csv} sowie je Lauf ein Protokoll."),
        ("cmd\\_odom\\_probe4.py",
         "Beobachtung der Befehlskette über vier Ebenen vom Regler bis zur "
         "gemessenen Bewegung. Grundlage des Befundes zum vorgeschalteten "
         "Drehbaustein."),
    ]),
    ("Auswertung \\quad \\texttt{auswertung/}", [
        ("anhang\\_a\\_tabellen.py",
         "Erzeugt die Tabellen in Anhang~\\ref{app:einzelwerte} aus den beiden "
         "Messdateien und leitet Drehung und Scherung aus den Wandwinkeln ab."),
        ("abbildungen\\_erzeugen.py",
         "Erzeugt die Belegungsgitter, das Streuungsdiagramm und die "
         "Kennwerte des Ergebniskapitels."),
        ("anhang\\_b\\_konfiguration.py",
         "Zieht die Konfigurationsauszüge für Anhang~\\ref{app:konfiguration} "
         "aus dem Repository."),
        ("anhang\\_cd\\_knoten.py",
         "Zieht die Quelltextauszüge für Anhang~\\ref{app:knoten} und erzeugt "
         "diese Tabelle."),
    ]),
]


def anhang_d(tag, commit, datum, url):
    stand = tag if tag else commit
    t = ["% Erzeugt von anhang_cd_knoten.py, nicht von Hand bearbeiten.", ""]
    t.append("Der vollständige Quelltext, die Konfigurationen und die "
             "Auswertungsskripte liegen im Repository")
    t.append("")
    t.append("\\begin{center}")
    t.append(f"\\texttt{{{tex_escape(url)}}}")
    t.append("\\end{center}")
    t.append("")
    t.append("Maßgeblich ist der Stand \\texttt{" + tex_escape(stand) + "} vom "
             + (datum or "siehe Repository")
             + ". Alle in dieser Arbeit berichteten Läufe beruhen auf diesem "
               "Stand. Der vorgelagerte Inflationsradius und die nachgelagerte "
               "Freiraumprüfung wurden dabei beim Start des Explorationsknotens "
               "als Laufzeitparameter übergeben und nicht als Vorgabe in der "
               "Datei gesetzt. Ihre für die Versuchsreihe maßgeblichen Werte "
               "nennt Tabelle~\\ref{tab:explorer_params}. Die folgende Übersicht "
               "ordnet den einzelnen Dateien ihre Aufgabe zu.")
    t.append("")
    t.append("\\begin{table}[htbp]")
    t.append("\\centering")
    t.append("\\small")
    t.append("\\caption{Verwendete Werkzeuge und ihre Aufgabe.}")
    t.append("\\label{tab:anh_werkzeuge}")
    t.append("\\begin{tabular}{@{}>{\\raggedright\\arraybackslash}p{4.4cm}>{\\raggedright\\arraybackslash}p{9.6cm}@{}}")
    t.append("\\toprule")
    t.append("Datei & Aufgabe \\\\")
    for gruppe, eintraege in WERKZEUGE:
        t.append("\\midrule")
        t.append(f"\\multicolumn{{2}}{{l}}{{\\textit{{{gruppe}}}}} \\\\")
        for datei, zweck in eintraege:
            t.append(f"{{\\scriptsize\\ttfamily {datei}}} & {zweck} \\\\")
            t.append("\\addlinespace[2pt]")
    t.append("\\bottomrule")
    t.append("\\end{tabular}")
    t.append("\\end{table}")
    return t


# --------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser()
    p.add_argument("--repo", default="~/go2_autonomy")
    p.add_argument("--out", default="pictures")
    p.add_argument("--codepfad", default="code")
    p.add_argument("--url", default="https://github.com/lukas32123/go2_autonomy")
    a = p.parse_args()

    repo = os.path.expanduser(a.repo)
    out = os.path.expanduser(a.out)
    if not os.path.isdir(repo):
        raise SystemExit(f"Repository nicht gefunden: {repo}")
    code = os.path.join(out, "code")
    os.makedirs(code, exist_ok=True)

    commit, tag, datum = stand_des_repos(repo)
    print(f"Repository {repo}")
    print(f"Stand      {tag or commit or 'unbekannt'}   {datum}")
    if not tag:
        print("  Hinweis: auf HEAD liegt kein Tag. Setze vor der Abgabe einen "
              "Tag auf den Stand der Messreihe.")
    print()

    quelle = lies(os.path.join(repo, "frontier", "frontier_explorer.py"))

    print("Auszuege aus frontier_explorer.py")
    detektion = kuerze_nach(methode(quelle, "map_cb"), "n_cells = sum(frontier)")
    pruefung = (methode(quelle, "has_clearance") + [""]
                + methode(quelle, "goal_still_valid"))
    auswahl = methode(quelle, "select_best")

    umfang = {}
    for name, block in [("explorer_detektion.py", detektion),
                        ("explorer_zielpruefung.py", pruefung),
                        ("explorer_zielauswahl.py", auswahl)]:
        umfang[name] = schreibe(os.path.join(code, name), block)

    # ---- Anhang C ----------------------------------------------------------
    tex = ["% Erzeugt von anhang_cd_knoten.py, nicht von Hand bearbeiten.",
           f"% Repository-Stand {tag or commit}", ""]

    def listing(datei, unterschrift, label):
        tex.append(f"\\lstinputlisting[language=Python, caption={{{unterschrift}}}, "
                   f"label={{{label}}}]{{{a.codepfad}/{datei}}}")
        tex.append("")

    listing("explorer_detektion.py",
            "Verarbeitung einer Kartennachricht mit der vorgelagerten "
            "Aufblähung und der Detektion der Grenzzellen. Der Auszug endet "
            "vor der Gruppierung.",
            "lst:detektion")
    listing("explorer_zielpruefung.py",
            "Geometrische Freiraumprüfung und laufende Prüfung des aktiven "
            "Ziels.",
            "lst:zielpruefung")
    listing("explorer_zielauswahl.py",
            "Bewertungsfunktion der Zielauswahl.",
            "lst:zielauswahl")

    ziel_c = os.path.join(out, "anhang_c_knoten.tex")
    io.open(ziel_c, "w", encoding="utf-8").write("\n".join(tex) + "\n")
    print(f"\n  geschrieben: {ziel_c}")

    # ---- Anhang D ----------------------------------------------------------
    ziel_d = os.path.join(out, "anhang_d_werkzeuge.tex")
    io.open(ziel_d, "w", encoding="utf-8").write(
        "\n".join(anhang_d(tag, commit, datum, a.url)) + "\n")
    print(f"  geschrieben: {ziel_d}")

    print(f"\n  Anhang C umfasst {sum(umfang.values())} Zeilen Quelltext, "
          f"das entspricht etwa {sum(umfang.values())/48:.0f} Seiten.")


if __name__ == "__main__":
    main()

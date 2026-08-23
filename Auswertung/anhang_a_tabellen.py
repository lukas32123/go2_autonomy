#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
anhang_a_tabellen.py

Erzeugt die vier LaTeX-Tabellen fuer Anhang A der Bachelorarbeit aus den
beiden Mess-CSV-Dateien. Ausgabe ist eine einzige .tex-Datei, deren Inhalt
in die main.tex eingefuegt wird.

Quellen
    eval_gesamt.csv        geschrieben von tools/eval_probe2.py, eine Zeile je Lauf
    probe_ergebnisse.csv   geschrieben von tools/eval_probe.py,  eine Zeile je Lauf

Beide Dateien enthalten neben den zwanzig Endlaeufen auch Entwicklungslaeufe.
Gefiltert wird deshalb ueber die feste Namensliste weiter unten und nicht
ueber ein Praefix. Der Lauf final_sr_1 ist ein Vorlauf und gehoert NICHT dazu.

Abhaengigkeiten: keine, nur die Standardbibliothek.

Aufruf
    python3 anhang_a_tabellen.py
"""

import csv
import os
import statistics as st
from decimal import Decimal, ROUND_HALF_UP

# ====================================================================
# PFADE, hier eintragen
# ====================================================================

CSV_EVAL = "/pfad/zu/eval_gesamt.csv"
CSV_PROBE = "/pfad/zu/probe_ergebnisse.csv"
TEX_AUS = "anhang_a_tabellen.tex"

# ====================================================================
# Die zwanzig Evaluierungslaeufe
# ====================================================================

LAEUFE_SR = [f"final_srm_{i}" for i in range(1, 11)]
LAEUFE_RF = [f"final_uni_{i}" for i in range(1, 11)]

UMGEBUNGEN = [
    ("Saeulenraum", "S\\\"aulenraum", LAEUFE_SR),
    ("Ringflur", "Ringflur", LAEUFE_RF),
]

# Zellgroesse des Belegungsgitters in Metern, aus mapper_params_go2.yaml
AUFLOESUNG = 0.05


# ====================================================================
# Hilfsfunktionen
# ====================================================================

def lies_csv(pfad):
    """Liest eine semikolongetrennte CSV in ein Dictionary ueber die Spalte name."""
    if not os.path.isfile(pfad):
        raise SystemExit(f"Datei nicht gefunden: {pfad}")
    with open(pfad, newline="", encoding="utf-8-sig") as fh:
        return {z["name"]: z for z in csv.DictReader(fh, delimiter=";")}


def pruefe(daten, laeufe, quelle):
    fehlend = [n for n in laeufe if n not in daten]
    if fehlend:
        raise SystemExit(f"In {quelle} fehlen folgende Laeufe: {', '.join(fehlend)}")


def zahl(wert, stellen=2):
    """Formatiert eine Zahl mit deutschem Dezimalkomma.

    Gerundet wird kaufmaennisch, also die Haelfte stets vom Nullpunkt weg.
    Die Standardrundung von Python entscheidet solche Grenzfaelle nach der
    binaeren Darstellung und wich damit an zwei Stellen von den bereits in
    Kapitel 4 berichteten Werten ab.
    """
    if wert is None or wert == "":
        return "--"
    q = Decimal(1).scaleb(-stellen)
    d = Decimal(repr(float(wert))).quantize(q, rounding=ROUND_HALF_UP)
    if d == 0:
        d = abs(d)
    return f"{d}".replace(".", ",")


def name_tex(n):
    """Setzt den Laufnamen als Schreibmaschinentext mit maskierten Unterstrichen."""
    return "\\texttt{" + n.replace("_", "\\_") + "}"


def statistikzeilen(spalten, stellen):
    """Erzeugt die beiden Zeilen Mittelwert und Standardabweichung."""
    mw = " & ".join(zahl(st.mean(v), s) for v, s in zip(spalten, stellen))
    sd = " & ".join(zahl(st.stdev(v), s) for v, s in zip(spalten, stellen))
    return mw, sd


def hole(daten, laeufe, spalte, typ=float):
    return [typ(daten[n][spalte]) for n in laeufe]


# ====================================================================
# Tabelle A.1, Explorationseffizienz je Lauf
# ====================================================================

def tabelle_effizienz(ev):
    z = []
    z.append("\\begin{table}[htbp]")
    z.append("\\centering")
    z.append("\\small")
    z.append("\\caption{Einzelwerte der Explorationseffizienz je Lauf. Die "
             "Spalte $d_{\\mathrm{odom}}$ ist der unabh\\\"angig aus der "
             "Odometrie gewonnene Gegenwert zur Gesamtdistanz.}")
    z.append("\\label{tab:anh_effizienz}")
    z.append("\\begin{tabular}{lrrrrrrrrc}")
    z.append("\\toprule")
    z.append("Lauf & Ziele & $t_{\\mathrm{erk}}$ & $t_{\\mathrm{r\\ddot{u}ck}}$ "
             "& $t_{\\mathrm{ges}}$ & $d_{\\mathrm{erk}}$ & "
             "$d_{\\mathrm{r\\ddot{u}ck}}$ & $d_{\\mathrm{ges}}$ & "
             "$d_{\\mathrm{odom}}$ & R\\\"uckkehr \\\\")
    z.append(" & & [s] & [s] & [s] & [m] & [m] & [m] & [m] & \\\\")

    for _klar, titel, laeufe in UMGEBUNGEN:
        z.append("\\midrule")
        z.append(f"\\multicolumn{{10}}{{l}}{{\\textit{{{titel}}}}} \\\\")
        for n in laeufe:
            r = ev[n]
            rueck = "ja" if r["rth"].strip().upper() == "ERFOLGREICH" else "nein"
            z.append(
                f"{name_tex(n)} & {r['ziele']} & {zahl(r['t_expl'], 1)} & "
                f"{zahl(r['t_rth'], 1)} & {zahl(r['t_ges'], 1)} & "
                f"{zahl(r['d_expl'])} & {zahl(r['d_rth'])} & "
                f"{zahl(r['d_ges'])} & {zahl(r['d_odom'])} & {rueck} \\\\"
            )
        spalten = [
            hole(ev, laeufe, "ziele"), hole(ev, laeufe, "t_expl"),
            hole(ev, laeufe, "t_rth"), hole(ev, laeufe, "t_ges"),
            hole(ev, laeufe, "d_expl"), hole(ev, laeufe, "d_rth"),
            hole(ev, laeufe, "d_ges"), hole(ev, laeufe, "d_odom"),
        ]
        # zwei Nachkommastellen, damit die Werte unmittelbar mit
        # Tabelle~\ref{tab:effizienz} im Hauptteil abgeglichen werden koennen
        stellen = [2, 2, 2, 2, 2, 2, 2, 2]
        mw, sd = statistikzeilen(spalten, stellen)
        z.append("\\cmidrule(lr){1-10}")
        z.append(f"Mittelwert & {mw} & \\\\")
        z.append(f"Standardabw. & {sd} & \\\\")

    z.append("\\bottomrule")
    z.append("\\end{tabular}")
    z.append("\\end{table}")
    return z


# ====================================================================
# Tabelle A.2, Zaehler der Fehlerbehandlung und Sicherung je Lauf
# ====================================================================

def tabelle_zaehler(ev):
    z = []
    z.append("\\begin{table}[htbp]")
    z.append("\\centering")
    z.append("\\footnotesize")
    z.append("\\caption{Einzelwerte der Fehlerbehandlungs- und "
             "Sicherungsz\\\"ahler je Lauf.}")
    z.append("\\label{tab:anh_zaehler}")
    z.append("\\begin{tabular}{lrrrrrrrrrrrr}")
    z.append("\\toprule")
    z.append("& & \\multicolumn{4}{c}{Knoten} & "
             "\\multicolumn{3}{c}{Wiederherstellung} & & & "
             "\\multicolumn{2}{c}{Kartenstrom} \\\\")
    z.append("\\cmidrule(lr){3-6} \\cmidrule(lr){7-9} \\cmidrule(lr){12-13}")
    z.append("Lauf & Planer & Sperr. & Reval. & Wdh. & Zeitl. & Drehen & "
             "Warten & Zur\\\"uck & Berein. & Koll. & verw. & ges. \\\\")

    felder = ["planer_fehler", "blacklist", "reval", "retry", "timeout",
              "spin", "wait", "backup", "costmap_clear", "collision_events",
              "map_malformed_nav2", "map_n"]

    for _klar, titel, laeufe in UMGEBUNGEN:
        z.append("\\midrule")
        z.append(f"\\multicolumn{{13}}{{l}}{{\\textit{{{titel}}}}} \\\\")
        for n in laeufe:
            r = ev[n]
            werte = " & ".join(str(int(r[f])) for f in felder)
            z.append(f"{name_tex(n)} & {werte} \\\\")
        summen = " & ".join(str(sum(hole(ev, laeufe, f, int))) for f in felder)
        z.append("\\cmidrule(lr){1-13}")
        z.append(f"Summe & {summen} \\\\")

    z.append("\\bottomrule")
    z.append("\\end{tabular}")
    z.append("\\end{table}")
    return z


# ====================================================================
# Tabelle A.3, Rumpflage, Nachfuehrung und Stillstaende je Lauf
# ====================================================================

def tabelle_lage(pr):
    z = []
    z.append("\\begin{table}[htbp]")
    z.append("\\centering")
    z.append("\\small")
    z.append("\\caption{Rumpflage, Bewegung der Nachf\\\"uhrung zwischen "
             "Kartenbezug und Odometriebezug sowie Stillst\\\"ande je Lauf.}")
    z.append("\\label{tab:anh_lage}")
    z.append("\\begin{tabular}{lrrrrrrrrr}")
    z.append("\\toprule")
    z.append("& \\multicolumn{2}{c}{Nickwinkel} & "
             "\\multicolumn{2}{c}{Rollwinkel} & "
             "\\multicolumn{3}{c}{Nachf\\\"uhrung} & "
             "\\multicolumn{2}{c}{Stillstand} \\\\")
    z.append("\\cmidrule(lr){2-3} \\cmidrule(lr){4-5} "
             "\\cmidrule(lr){6-8} \\cmidrule(lr){9-10}")
    z.append("Lauf & min & max & min & max & $\\Delta x$ & $\\Delta y$ & "
             "$\\Delta\\psi$ & Anzahl & l\\\"angster \\\\")
    z.append("& [$^\\circ$] & [$^\\circ$] & [$^\\circ$] & [$^\\circ$] & "
             "[cm] & [cm] & [$^\\circ$] & & [s] \\\\")

    for _klar, titel, laeufe in UMGEBUNGEN:
        z.append("\\midrule")
        z.append(f"\\multicolumn{{10}}{{l}}{{\\textit{{{titel}}}}} \\\\")
        for n in laeufe:
            r = pr[n]
            z.append(
                f"{name_tex(n)} & {zahl(r['nick_min'])} & {zahl(r['nick_max'])} & "
                f"{zahl(r['roll_min'])} & {zahl(r['roll_max'])} & "
                f"{zahl(r['mo_tx_cm'], 1)} & {zahl(r['mo_ty_cm'], 1)} & "
                f"{zahl(r['mo_yaw_deg'])} & {int(r['stalls'])} & "
                f"{zahl(r['stall_max_s'], 1)} \\\\"
            )
        spalten = [
            hole(pr, laeufe, "nick_min"), hole(pr, laeufe, "nick_max"),
            hole(pr, laeufe, "roll_min"), hole(pr, laeufe, "roll_max"),
            hole(pr, laeufe, "mo_tx_cm"), hole(pr, laeufe, "mo_ty_cm"),
            hole(pr, laeufe, "mo_yaw_deg"), hole(pr, laeufe, "stalls"),
            hole(pr, laeufe, "stall_max_s"),
        ]
        stellen = [2, 2, 2, 2, 1, 1, 2, 1, 1]
        mw, sd = statistikzeilen(spalten, stellen)
        z.append("\\cmidrule(lr){1-10}")
        z.append(f"Mittelwert & {mw} \\\\")
        z.append(f"Standardabw. & {sd} \\\\")

    z.append("\\bottomrule")
    z.append("\\end{tabular}")
    z.append("\\end{table}")
    return z


# ====================================================================
# Tabelle A.4, Kartenverzerrung je Lauf
# ====================================================================

def tabelle_verzerrung(pr):
    z = []
    z.append("\\begin{table}[htbp]")
    z.append("\\centering")
    z.append("\\small")
    z.append("\\caption{Gemessene Wandwinkel, daraus abgeleitete Drehung und "
             "Scherung sowie Kartengr\\\"o\\ss{}e je Lauf. Die Zerlegung folgt "
             "Gleichung~\\ref{eq:zerlegung}.}")
    z.append("\\label{tab:anh_verzerrung}")
    z.append("\\begin{tabular}{lrrrrrr}")
    z.append("\\toprule")
    z.append("Lauf & $\\alpha_{\\mathrm{l}}$ & $\\alpha_{\\mathrm{o}}$ & "
             "$\\theta$ & $\\gamma$ & Karte & Kante \\\\")
    z.append("& [$^\\circ$] & [$^\\circ$] & [$^\\circ$] & [$^\\circ$] & "
             "[Zellen] & [m] \\\\")

    for _klar, titel, laeufe in UMGEBUNGEN:
        z.append("\\midrule")
        z.append(f"\\multicolumn{{7}}{{l}}{{\\textit{{{titel}}}}} \\\\")
        thetas, gammas = [], []
        for n in laeufe:
            r = pr[n]
            al = float(r["shear_links"])
            ao = float(r["shear_oben"])
            theta = (ao - al) / 2.0
            gamma = (ao + al) / 2.0
            thetas.append(theta)
            gammas.append(gamma)
            b, h = int(r["karte_b"]), int(r["karte_h"])
            kante = max(b, h) * AUFLOESUNG
            z.append(
                f"{name_tex(n)} & {zahl(al, 1)} & {zahl(ao, 1)} & "
                f"{zahl(theta)} & {zahl(gamma)} & "
                f"${b} \\times {h}$ & {zahl(kante, 1)} \\\\"
            )
        spalten = [hole(pr, laeufe, "shear_links"),
                   hole(pr, laeufe, "shear_oben"), thetas, gammas]
        mw, sd = statistikzeilen(spalten, [1, 1, 2, 2])
        z.append("\\cmidrule(lr){1-7}")
        z.append(f"Mittelwert & {mw} & & \\\\")
        z.append(f"Standardabw. & {sd} & & \\\\")

    z.append("\\bottomrule")
    z.append("\\end{tabular}")
    z.append("\\end{table}")
    return z


# ====================================================================
# Kontrollausgabe auf der Konsole
# ====================================================================

def kontrolle(ev, pr):
    print("Kontrollwerte zum Abgleich mit Kapitel 4")
    print("-" * 60)
    for titel, _tex, laeufe in UMGEBUNGEN:
        tg = hole(ev, laeufe, "t_ges")
        dg = hole(ev, laeufe, "d_ges")
        al = hole(pr, laeufe, "shear_links")
        ao = hole(pr, laeufe, "shear_oben")
        th = [(o - l) / 2 for l, o in zip(al, ao)]
        ga = [(o + l) / 2 for l, o in zip(al, ao)]
        print(f"{titel}")
        print(f"  Gesamtzeit      {st.mean(tg):8.2f} s   "
              f"SD {st.stdev(tg):6.2f}   "
              f"VK {100 * st.stdev(tg) / st.mean(tg):5.1f} %")
        print(f"  Gesamtdistanz   {st.mean(dg):8.2f} m   "
              f"SD {st.stdev(dg):6.2f}   "
              f"VK {100 * st.stdev(dg) / st.mean(dg):5.1f} %")
        print(f"  Drehung         {st.mean(th):8.2f} Grad SD {st.stdev(th):6.2f}")
        print(f"  Scherung        {st.mean(ga):8.2f} Grad SD {st.stdev(ga):6.2f}")
        print(f"  Bereinigungen   {sum(hole(ev, laeufe, 'costmap_clear', int)):4d}"
              f"   Kollisionen {sum(hole(ev, laeufe, 'collision_events', int)):4d}")
        print(f"  Karten verworfen {sum(hole(ev, laeufe, 'map_malformed_nav2', int)):3d}"
              f"  von {sum(hole(ev, laeufe, 'map_n', int)):5d}")
        print()


# ====================================================================
# Hauptteil
# ====================================================================

def main():
    ev = lies_csv(CSV_EVAL)
    pr = lies_csv(CSV_PROBE)

    alle = LAEUFE_SR + LAEUFE_RF
    pruefe(ev, alle, os.path.basename(CSV_EVAL))
    pruefe(pr, alle, os.path.basename(CSV_PROBE))

    bloecke = []
    bloecke += ["% ----- Tabelle A.1 -----"] + tabelle_effizienz(ev) + [""]
    bloecke += ["% ----- Tabelle A.2 -----"] + tabelle_zaehler(ev) + [""]
    bloecke += ["% ----- Tabelle A.3 -----"] + tabelle_lage(pr) + [""]
    bloecke += ["% ----- Tabelle A.4 -----"] + tabelle_verzerrung(pr) + [""]

    kopf = [
        "% Erzeugt von anhang_a_tabellen.py, nicht von Hand bearbeiten.",
        f"% Quellen: {os.path.basename(CSV_EVAL)}, {os.path.basename(CSV_PROBE)}",
        "% Grundlage sind die zwanzig Evaluierungslaeufe final_srm_1 bis 10",
        "% und final_uni_1 bis 10.",
        "",
    ]

    with open(TEX_AUS, "w", encoding="utf-8") as fh:
        fh.write("\n".join(kopf + bloecke))

    print(f"Geschrieben: {os.path.abspath(TEX_AUS)}")
    print()
    kontrolle(ev, pr)


if __name__ == "__main__":
    main()

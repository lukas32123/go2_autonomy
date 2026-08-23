#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Erzeugt die datengestuetzten Abbildungen und die Kennzahlen fuer Kapitel 4.

Aufruf im Ordner, in dem eval_gesamt.csv und probe_ergebnisse.csv liegen:

    python3 abbildungen_erzeugen.py --basis ~/welt2_slam/maps/eval \
                                    --out   pictures

Unter --basis wird rekursiv gesucht, die Ablage in Unterordnern je Lauf ist
also kein Problem. Erkannt werden Karten als <lauf>.pgm und Protokolle als
<lauf>_explorer.log oder expl_<lauf>.log.

Benoetigt: numpy, matplotlib.   pip install numpy matplotlib
"""

import argparse, csv, io, math, os, re, sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

# --------------------------------------------------------------------------
# Konstanten der beiden Testumgebungen (aus den Szenenskripten)
# --------------------------------------------------------------------------
UMGEBUNG = {
    "sr":  dict(titel="Säulenraum", praefix="final_srm_",
                aussenmass=20.2,          # Kantenlaenge inkl. Wandstaerke [m]
                startpose=(0.00, -8.70)), # Startpose im Weltbezug [m]
    "uni": dict(titel="Ringflur",   praefix="final_uni_",
                aussenmass=30.2,
                startpose=(0.00, -11.50)),
}
RES = 0.05          # Kartenaufloesung [m/Zelle]
T975_DF9 = 2.262    # t-Quantil, zweiseitig 95 Prozent, 9 Freiheitsgrade


# --------------------------------------------------------------------------
def lade(csv_gesamt, csv_probe):
    ges = {r["name"]: r for r in csv.DictReader(
        io.open(csv_gesamt, encoding="utf-8-sig"), delimiter=";")}
    pro = {r["name"]: r for r in csv.DictReader(
        io.open(csv_probe, encoding="utf-8-sig"), delimiter=";")}
    return ges, pro


def laeufe(ges, praefix):
    n = [k for k in ges if k.startswith(praefix)]
    return sorted(n, key=lambda x: int(x.rsplit("_", 1)[1]))


def kennwerte(ges, pro, namen):
    """Liefert je Lauf ein Wörterbuch mit den fünf Kennwerten."""
    d = {}
    for n in namen:
        g, p = ges[n], pro[n]
        al = float(p["shear_links"])      # Winkel der linken Wand
        ao = float(p["shear_oben"])       # Winkel der oberen Wand
        d[n] = dict(
            t_ges=float(g["t_ges"]),  t_expl=float(g["t_expl"]), t_rth=float(g["t_rth"]),
            d_ges=float(g["d_ges"]),  d_expl=float(g["d_expl"]), d_rth=float(g["d_rth"]),
            ziele=float(g["ziele"]),
            theta=(ao - al) / 2.0,        # Drehung
            gamma=(ao + al) / 2.0,        # Scherung
        )
    return d


def stat(werte):
    m = float(np.mean(werte)); s = float(np.std(werte, ddof=1))
    hw = T975_DF9 * s / math.sqrt(len(werte))
    return m, s, hw


# --------------------------------------------------------------------------
def teil_a_kennzahlen(ges, pro):
    """Zahlen fuer Schritt 33 und fuer Abschnitt 4.2.3."""
    print("\n" + "=" * 78)
    print("TEIL A  Kennzahlen fuer Tabelle 4.1 und Abschnitt 4.2.3")
    print("=" * 78)
    for kuerzel, u in UMGEBUNG.items():
        namen = laeufe(ges, u["praefix"])
        k = kennwerte(ges, pro, namen)
        print(f"\n{u['titel']}   n = {len(namen)}")
        print(f"  {'Groesse':<22}{'Mittel':>9}{'SD':>9}{'VarKo':>8}"
              f"{'KI-Halbweite':>15}{'rel.':>8}")
        for feld, name, eh in [
            ("t_expl", "Erkundungszeit",     "s"),
            ("t_rth",  "Rueckkehrzeit",      "s"),
            ("t_ges",  "Gesamtzeit",         "s"),
            ("d_expl", "Erkundungsdistanz",  "m"),
            ("d_rth",  "Rueckkehrdistanz",   "m"),
            ("d_ges",  "Gesamtdistanz",      "m"),
            ("ziele",  "Ziele",              ""),
        ]:
            v = [k[n][feld] for n in namen]
            m, s, hw = stat(v)
            print(f"  {name+' ['+eh+']':<22}{m:9.2f}{s:9.2f}"
                  f"{s/m*100:7.1f}%{hw:15.2f}{hw/m*100:7.1f}%")
        for feld, name in [("theta", "Drehung [Grad]"), ("gamma", "Scherung [Grad]")]:
            v = [k[n][feld] for n in namen]
            m, s, _ = stat(v)
            print(f"  {name:<22}{m:9.3f}{s:9.3f}")
        # Anteil der Rueckkehr
        tr = np.mean([k[n]["t_rth"] for n in namen]) / np.mean([k[n]["t_ges"] for n in namen])
        dr = np.mean([k[n]["d_rth"] for n in namen]) / np.mean([k[n]["d_ges"] for n in namen])
        print(f"  Anteil der Rueckkehr an der Zeit     {tr*100:5.1f} Prozent")
        print(f"  Anteil der Rueckkehr an der Distanz  {dr*100:5.1f} Prozent")


def teil_b_repraesentativ(ges, pro):
    """Waehlt je Umgebung den repraesentativsten Lauf."""
    print("\n" + "=" * 78)
    print("TEIL B  Auswahl des repraesentativen Laufs")
    print("=" * 78)
    gewaehlt = {}
    for kuerzel, u in UMGEBUNG.items():
        namen = laeufe(ges, u["praefix"])
        k = kennwerte(ges, pro, namen)
        felder = ["t_ges", "d_ges", "ziele", "theta", "gamma"]
        M = {f: np.mean([k[n][f] for n in namen]) for f in felder}
        S = {f: np.std([k[n][f] for n in namen], ddof=1) for f in felder}
        rang = []
        for n in namen:
            z = {f: (k[n][f] - M[f]) / S[f] if S[f] > 0 else 0.0 for f in felder}
            rang.append((sum(abs(z[f]) for f in felder), n, z))
        rang.sort()
        gewaehlt[kuerzel] = rang[0][1]
        print(f"\n{u['titel']}")
        print(f"  {'Lauf':<16}{'Summe |z|':>10}   Einzelabweichungen")
        for s, n, z in rang:
            det = "  ".join(f"{f}={z[f]:+.2f}" for f in felder)
            mark = "  <== gewaehlt" if n == rang[0][1] else ""
            print(f"  {n:<16}{s:10.2f}   {det}{mark}")
    return gewaehlt



# --------------------------------------------------------------------------
def finde_datei(basis, muster_liste):
    """Sucht rekursiv unter basis. Muster werden der Reihe nach probiert,
    das erste Muster mit Treffern gewinnt."""
    import fnmatch
    for m in muster_liste:
        treffer = []
        for wurzel, _, dateien in os.walk(basis):
            for d in dateien:
                if fnmatch.fnmatch(d.lower(), m.lower()):
                    treffer.append(os.path.join(wurzel, d))
        if treffer:
            treffer.sort()
            return treffer[0]
    return None


def pfade_fuer(basis, run):
    """Liefert (pgm, log) fuer einen Lauf, oder None wo nichts gefunden wurde."""
    pgm = finde_datei(basis, [f"{run}.pgm", f"{run}_*.pgm", f"*{run}*.pgm"])
    log = finde_datei(basis, [f"{run}_explorer.log", f"expl_{run}.log",
                              f"*{run}*explor*.log"])
    return pgm, log


def teil_null_dateien(basis, gewaehlt):
    """Zeigt, welche Dateien gefunden wurden."""
    print("\n" + "=" * 78)
    print("TEIL 0  Gefundene Dateien unter", os.path.abspath(basis))
    print("=" * 78)
    gefunden = {}
    for kuerzel, u in UMGEBUNG.items():
        run = gewaehlt[kuerzel]
        pgm, log = pfade_fuer(basis, run)
        gefunden[kuerzel] = (pgm, log)
        print(f"\n{u['titel']}  Lauf {run}")
        print(f"  Karte      {pgm if pgm else 'NICHT GEFUNDEN'}")
        print(f"  Protokoll  {log if log else 'NICHT GEFUNDEN'}")
    return gefunden


# --------------------------------------------------------------------------
def lies_pgm(pfad):
    """Liest ein PGM im Binaerformat P5 und liefert ein Array der Form (h, w)."""
    with open(pfad, "rb") as f:
        roh = f.read()
    if not roh.startswith(b"P5"):
        raise ValueError("kein binaeres PGM")
    felder, i = [], 2
    while len(felder) < 3:
        while i < len(roh) and roh[i:i+1].isspace():
            i += 1
        if roh[i:i+1] == b"#":
            while roh[i:i+1] not in (b"\n", b""):
                i += 1
            continue
        j = i
        while j < len(roh) and not roh[j:j+1].isspace():
            j += 1
        felder.append(int(roh[i:j])); i = j
    i += 1
    w, h, _ = felder
    return np.frombuffer(roh[i:i+w*h], dtype=np.uint8).reshape(h, w)


def zeichne_karte(ax, bild):
    """Zeichnet eine Karte in map_saver-Kodierung als Graustufenbild."""
    darst = np.full(bild.shape, 0.72)      # unbekannt
    darst[bild >= 250] = 1.00              # frei
    darst[bild <= 10]  = 0.10              # belegt
    ax.imshow(darst, cmap="gray", vmin=0, vmax=1, interpolation="nearest")
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_edgecolor("0.4")


def massstab(ax, bild, laenge_m=5.0):
    """Setzt einen Massstabsbalken in die linke untere Ecke."""
    px = laenge_m / RES
    h, w = bild.shape
    x0, y0 = 0.04 * w, 0.94 * h
    ax.plot([x0, x0 + px], [y0, y0], color="black", lw=2.5, solid_capstyle="butt")
    ax.text(x0 + px / 2, y0 - 0.025 * h, f"{laenge_m:.0f} m",
            ha="center", va="bottom", fontsize=8)


def teil_c_karten(gefunden, out, gewaehlt):
    print("\n" + "=" * 78)
    print("TEIL C  Karten der gewaehlten Laeufe")
    print("=" * 78)
    for kuerzel, u in UMGEBUNG.items():
        name = gewaehlt[kuerzel]
        pfad = gefunden[kuerzel][0]
        if not pfad:
            print(f"  uebersprungen, keine Karte fuer {name}"); continue
        bild = lies_pgm(pfad)
        fig, ax = plt.subplots(figsize=(4.4, 4.4 * bild.shape[0] / bild.shape[1]))
        zeichne_karte(ax, bild)
        massstab(ax, bild)
        # Auswertebaender bei 20 bis 30 und 70 bis 80 Prozent der Hoehe
        h, w = bild.shape
        for a, b in [(0.20, 0.30), (0.70, 0.80)]:
            ax.axhspan(a * h, b * h, color="black", alpha=0.10, lw=0)
        fig.tight_layout(pad=0.2)
        ziel = os.path.join(out, f"karte_{'saeulenraum' if kuerzel=='sr' else 'ringflur'}.png")
        fig.savefig(ziel, dpi=300); plt.close(fig)
        print(f"  geschrieben: {ziel}   ({w} x {h} Zellen, Lauf {name})")


def teil_d_ausschnitt(gefunden, out, gewaehlt, mitte_m=None, kante_m=6.0):
    """Vergroesserter Ausschnitt aus der Saeulenraum-Karte."""
    print("\n" + "=" * 78)
    print("TEIL D  Ausschnitt fuer die offenen Saeulenseiten")
    print("=" * 78)
    pfad = gefunden["sr"][0]
    if not pfad:
        print("  uebersprungen, keine Karte fuer den Saeulenraum"); return
    bild = lies_pgm(pfad)
    h, w = bild.shape
    k = int(kante_m / RES)
    cx, cy = (w // 2, h // 2) if mitte_m is None else mitte_m
    x0, y0 = max(0, cx - k // 2), max(0, cy - k // 2)
    aus = bild[y0:y0 + k, x0:x0 + k]

    fig, axes = plt.subplots(1, 2, figsize=(8.0, 4.2))
    zeichne_karte(axes[0], bild)
    axes[0].add_patch(plt.Rectangle((x0, y0), k, k, fill=False, lw=1.4, ec="black"))
    axes[0].set_title("vollständige Karte", fontsize=9)
    zeichne_karte(axes[1], aus)
    axes[1].set_title(f"Ausschnitt, {kante_m:.0f} m Kantenlänge", fontsize=9)
    massstab(axes[1], aus, laenge_m=1.0)
    fig.tight_layout(pad=0.3)
    ziel = os.path.join(out, "saeulenluecke.png")
    fig.savefig(ziel, dpi=300); plt.close(fig)
    print(f"  geschrieben: {ziel}")
    print(f"  Mittelpunkt des Ausschnitts in Zellen: ({cx}, {cy}).")
    print("  Verschiebe ihn mit --zoom-mitte SPALTE ZEILE auf eine Saeule mit offener Seite.")


def teil_e_streuung(ges, pro, out):
    print("\n" + "=" * 78)
    print("TEIL E  Streuungsdiagramm")
    print("=" * 78)
    fig, axes = plt.subplots(2, 1, figsize=(6.2, 3.6), sharex=False)
    for ax, feld, titel, eh in [
            (axes[0], "t_ges", "Gesamtzeit",     "s"),
            (axes[1], "d_ges", "Gesamtdistanz",  "m")]:
        for zeile, (kuerzel, u) in enumerate(UMGEBUNG.items()):
            namen = laeufe(ges, u["praefix"])
            k = kennwerte(ges, pro, namen)
            v = np.array([k[n][feld] for n in namen])
            y = np.full_like(v, 1 - zeile, dtype=float)
            m, s, hw = stat(v)
            ax.hlines(1 - zeile, m - hw, m + hw, color="black", lw=6, alpha=0.18,
                      zorder=1)
            ax.plot(v, y + np.random.uniform(-0.06, 0.06, v.size), "o",
                    ms=4.5, mfc="white", mec="black", mew=0.9, zorder=3)
            ax.plot([m], [1 - zeile], "|", ms=18, color="black", mew=1.8, zorder=4)
            ax.text(m, 1 - zeile + 0.20, f"M = {m:.1f} {eh}", ha="center",
                    fontsize=7.5)
        ax.set_yticks([1, 0])
        ax.set_yticklabels([UMGEBUNG["sr"]["titel"], UMGEBUNG["uni"]["titel"]],
                           fontsize=8)
        ax.set_ylim(-0.5, 1.5)
        ax.set_xlabel(f"{titel} in {eh}", fontsize=8.5)
        ax.tick_params(labelsize=8)
        ax.grid(axis="x", color="0.9")
        ax.set_axisbelow(True)
        for s_ in ("top", "right", "left"):
            ax.spines[s_].set_visible(False)
    fig.tight_layout(pad=0.4)
    ziel = os.path.join(out, "streuung.pdf")
    fig.savefig(ziel); plt.close(fig)
    print(f"  geschrieben: {ziel}")
    print("  Der graue Balken ist das 95-Prozent-Konfidenzintervall des Mittelwertes.")


def teil_f_zielfolge(gefunden, out, gewaehlt):
    """Zeichnet die Abfolge der angesteuerten Ziele ueber der erzeugten Karte."""
    print("\n" + "=" * 78)
    print("TEIL F  Zielfolge ueber der Karte")
    print("=" * 78)
    re_ziel = re.compile(r"\[3b\] Ziel #(\d+):\s*\(([-\d.]+),\s*([-\d.]+)\)")
    re_home = re.compile(r"RETURN-TO-HOME.*?Startpose \(([-\d.]+),\s*([-\d.]+)\)")

    for kuerzel, u in UMGEBUNG.items():
        name = gewaehlt[kuerzel]
        pgm, log = gefunden[kuerzel]
        if not (log and pgm):
            print(f"  uebersprungen, Karte oder Protokoll fehlt fuer {name}"); continue
        txt = io.open(log, encoding="utf-8", errors="ignore").read()
        ziele = [(int(a), float(x), float(y)) for a, x, y in re_ziel.findall(txt)]
        heim = re_home.findall(txt)
        heim = (float(heim[0][0]), float(heim[0][1])) if heim else (0.0, 0.0)
        if not ziele:
            print(f"  keine Ziele in {log} gefunden"); continue

        bild = lies_pgm(pgm)
        h, w = bild.shape
        # Pixel-zu-Karte-Abbildung ueber die aeusseren Waende
        belegt = np.argwhere(bild <= 10)
        if belegt.size == 0:
            print(f"  keine belegten Zellen in {pgm}"); continue
        r0, c0 = belegt.min(axis=0); r1, c1 = belegt.max(axis=0)
        cx_px, cy_px = (c0 + c1) / 2.0, (r0 + r1) / 2.0
        # Raummitte im Kartenbezug = negative Startpose
        mx, my = -u["startpose"][0], -u["startpose"][1]

        def nach_px(x, y):
            return cx_px + (x - mx) / RES, cy_px - (y - my) / RES

        fig, ax = plt.subplots(figsize=(5.0, 5.0 * h / w))
        zeichne_karte(ax, bild)
        massstab(ax, bild)
        px = [nach_px(x, y) for _, x, y in ziele]
        sx = nach_px(heim[0], heim[1])
        ax.plot([sx[0]] + [p[0] for p in px], [sx[1]] + [p[1] for p in px],
                "-", color="black", lw=1.1, alpha=0.75, zorder=3)
        ax.plot([px[-1][0], sx[0]], [px[-1][1], sx[1]], "--", color="black",
                lw=1.1, alpha=0.75, zorder=3)
        for (nr, _, _), (X, Y) in zip(ziele, px):
            ax.plot(X, Y, "o", ms=11, mfc="white", mec="black", mew=1.1, zorder=4)
            ax.text(X, Y, str(nr), ha="center", va="center", fontsize=7, zorder=5)
        ax.plot(sx[0], sx[1], "s", ms=8, mfc="black", mec="black", zorder=4)
        ax.text(sx[0], sx[1] - 0.03 * h, "Start", ha="center", va="bottom",
                fontsize=7.5, zorder=5)
        fig.tight_layout(pad=0.2)
        ziel_datei = os.path.join(
            out, f"zielfolge_{'saeulenraum' if kuerzel=='sr' else 'ringflur'}.pdf")
        fig.savefig(ziel_datei); plt.close(fig)
        print(f"  geschrieben: {ziel_datei}   ({len(ziele)} Ziele, Lauf {name})")


# --------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser()
    p.add_argument("--gesamt", default="eval_gesamt.csv")
    p.add_argument("--probe",  default="probe_ergebnisse.csv")
    p.add_argument("--basis", default=".",
                   help="Ordner, unter dem rekursiv nach .pgm und _explorer.log gesucht wird")
    p.add_argument("--out",     default="pictures")
    p.add_argument("--zoom-mitte", nargs=2, type=int, default=None,
                   metavar=("SPALTE", "ZEILE"))
    p.add_argument("--zoom-kante", type=float, default=6.0)
    a = p.parse_args()

    os.makedirs(a.out, exist_ok=True)
    ges, pro = lade(a.gesamt, a.probe)

    teil_a_kennzahlen(ges, pro)
    gewaehlt = teil_b_repraesentativ(ges, pro)
    gefunden = teil_null_dateien(a.basis, gewaehlt)
    teil_c_karten(gefunden, a.out, gewaehlt)
    teil_d_ausschnitt(gefunden, a.out, gewaehlt,
                      mitte_m=tuple(a.zoom_mitte) if a.zoom_mitte else None,
                      kante_m=a.zoom_kante)
    teil_e_streuung(ges, pro, a.out)
    teil_f_zielfolge(gefunden, a.out, gewaehlt)
    print("\nFertig.")


if __name__ == "__main__":
    main()

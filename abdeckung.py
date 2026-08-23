#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
abdeckung.py -- misst die Kartenabdeckung der Evaluierungslaeufe.

Grundgedanke: Die wahre freie Grundflaeche beider Testumgebungen ist bekannt,
weil beide Szenen deterministisch aus einem Skript erzeugt werden. Sie wird hier
aus denselben Konstanten analytisch berechnet, die auch die Szene aufbauen.
Aus den je Lauf abgelegten Karten wird die als frei erkannte Flaeche gezaehlt.
Ihr Verhaeltnis ist die Abdeckung. Zusammen mit Zeit und Distanz aus
eval_gesamt.csv ergibt sich daraus der flaechenbezogene Erkundungsaufwand.

Aufruf:
    python3 abdeckung.py --karten ~/welt2_slam/maps/eval \
                         --eval   ~/welt2_slam/maps/eval/eval_gesamt.csv \
                         --out    abdeckung.csv

Abhaengigkeiten: keine, nur die Standardbibliothek.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path

RES = 0.05          # Kantenlaenge einer Zelle [m], wie in mapper_params_go2.yaml
FREI = 254          # PGM-Grauwert freier Zellen (map_saver-Konvention)
BELEGT = 0
UNBEKANNT = 205


# --------------------------------------------------------------------------
# Wahre Freiflaechen, analytisch aus den Szenenkonstanten
# --------------------------------------------------------------------------
def freiflaeche_ringflur() -> float:
    OUTER, CORR_WIDE, CORR_NARROW = 30.0, 6.0, 3.0
    ND, NW, PW, PN, CM, WT = 0.40, 1.40, 6.0, 4.0, 1.5, 0.20
    half = OUTER / 2.0
    back = half - WT / 2.0
    pier = back - ND
    core_y = pier - CORR_WIDE
    core_x = pier - CORR_NARROW
    core_solid = core_x - ND

    def centers(length, pitch):
        lim = length / 2.0 - CM - NW / 2.0
        out, k = [], 0
        while k * pitch <= lim:
            out.extend([0.0] if k == 0 else [-k * pitch, k * pitch])
            k += 1
        return sorted(out)

    def spans(length, cs):
        out, lo = [], -length / 2.0
        for c in cs:
            hi = c - NW / 2.0
            if hi - lo > 0.05:
                out.append((lo, hi))
            lo = c + NW / 2.0
        if length / 2.0 - lo > 0.05:
            out.append((lo, length / 2.0))
        return out

    gesamt = (2.0 * back) ** 2
    kern = (2.0 * core_solid) * (2.0 * core_y)
    pfeiler = 0.0
    for _ in (1, -1):
        for a, b in spans(OUTER, centers(OUTER, PW)):
            pfeiler += (b - a) * ND
        for a, b in spans(OUTER, centers(OUTER, PN)):
            pfeiler += ND * (b - a)
    core_len = 2.0 * core_y
    for _ in (1, -1):
        for a, b in spans(core_len, centers(core_len, PN)):
            pfeiler += ND * (b - a)
    return gesamt - kern - pfeiler


def freiflaeche_saeulenraum() -> tuple[float, int]:
    ROOM, WT, PIL, PITCH, JIT, EM, SC = 20.0, 0.20, 0.60, 2.50, 0.35, 1.60, 1.80
    inner = ROOM / 2.0 - WT / 2.0
    sx, sy = 0.0, -(inner - 1.20)

    def jitter(i, j, salt):
        v = math.sin(i * 127.1 + j * 311.7 + salt * 74.7) * 43758.5453
        return (v - math.floor(v)) * 2.0 - 1.0

    lim = inner - EM - PIL / 2.0 - JIT
    ks, k = [], 0
    while k * PITCH <= lim:
        ks.extend([0] if k == 0 else [-k, k])
        k += 1
    ks.sort()
    n = 0
    for i in ks:
        for j in ks:
            px = i * PITCH + JIT * jitter(i, j, 1.0)
            py = j * PITCH + JIT * jitter(i, j, 2.0)
            if math.hypot(px - sx, py - sy) < SC:
                continue
            n += 1
    return (2.0 * inner) ** 2 - n * PIL * PIL, n


# --------------------------------------------------------------------------
# PGM einlesen
# --------------------------------------------------------------------------
def lies_pgm(pfad: Path) -> list[int]:
    """Liest ein PGM (P2 oder P5) und gibt die Grauwerte als flache Liste."""
    roh = pfad.read_bytes()
    felder, i = [], 0
    while len(felder) < 4:
        while i < len(roh) and roh[i:i + 1].isspace():
            i += 1
        if roh[i:i + 1] == b"#":
            while i < len(roh) and roh[i:i + 1] not in (b"\n", b"\r"):
                i += 1
            continue
        j = i
        while j < len(roh) and not roh[j:j + 1].isspace():
            j += 1
        felder.append(roh[i:j])
        i = j
    magic = felder[0].decode()
    if magic not in ("P2", "P5"):
        raise SystemExit(f"{pfad.name}: kein PGM (Kennung {magic})")
    breite, hoehe = int(felder[1]), int(felder[2])
    i += 1                                   # genau ein Trennzeichen nach maxval
    if magic == "P5":
        werte = list(roh[i:i + breite * hoehe])
    else:
        werte = [int(x) for x in roh[i:].split()]
    if len(werte) != breite * hoehe:
        raise SystemExit(f"{pfad.name}: {len(werte)} Werte, erwartet {breite*hoehe}")
    return werte


# --------------------------------------------------------------------------
def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--karten", type=Path, required=True,
                   help="Wurzelordner, wird rekursiv nach PGM-Dateien durchsucht.")
    p.add_argument("--eval", type=Path, required=True, help="eval_gesamt.csv")
    p.add_argument("--out", type=Path, default=Path("abdeckung.csv"))
    p.add_argument("--praefix", default="final_", help="Nur Laeufe mit diesem Praefix.")
    a = p.parse_args()

    wahr = {"uni": freiflaeche_ringflur()}
    sr_flaeche, n_saeulen = freiflaeche_saeulenraum()
    wahr["sr"] = sr_flaeche
    print(f"Wahre Freiflaeche  Ringflur    {wahr['uni']:8.2f} m^2")
    print(f"Wahre Freiflaeche  Saeulenraum {wahr['sr']:8.2f} m^2  ({n_saeulen} Saeulen)")
    print()

    # eval_gesamt.csv einlesen (Semikolon getrennt)
    with a.eval.expanduser().open(encoding="utf-8-sig", newline="") as f:
        laeufe = {r["name"]: r for r in csv.DictReader(f, delimiter=";")}

    # Karten liegen je Lauf in einem eigenen Unterordner, deshalb rekursiv suchen.
    wurzel = a.karten.expanduser()
    gefunden = {}
    for pgm in sorted(wurzel.rglob(f"{a.praefix}*.pgm")):
        gefunden.setdefault(pgm.stem, pgm)
    if not gefunden:
        raise SystemExit(f"Keine PGM-Dateien unter {wurzel} gefunden.")
    print(f"{len(gefunden)} Karten unter {wurzel} gefunden.\n")

    zeilen = []
    for name, pgm in sorted(gefunden.items()):
        if name not in laeufe:
            print(f"[WARNUNG] {name}: kein Eintrag in eval_gesamt.csv, uebersprungen")
            continue
        r = laeufe[name]
        szene = r["szene"].strip()
        if szene not in wahr:
            print(f"[WARNUNG] {name}: unbekannte Szene '{szene}', uebersprungen")
            continue
        try:
            werte = lies_pgm(pgm)
        except PermissionError:
            raise SystemExit(
                f"Keine Leseberechtigung fuer {pgm}.\n"
                f"    Die Dateien wurden im Container als root angelegt. Abhilfe:\n"
                f"    sudo chown -R $USER:$USER {wurzel}")
        n_frei = sum(1 for v in werte if v == FREI)
        flaeche = n_frei * RES * RES
        abdeckung = 100.0 * flaeche / wahr[szene]
        d_ges = float(r["d_ges"].replace(",", "."))
        t_ges = float(r["t_ges"].replace(",", "."))
        zeilen.append({
            "name": name, "szene": szene,
            "frei_m2": round(flaeche, 2),
            "abdeckung_pct": round(abdeckung, 2),
            "m2_pro_m": round(flaeche / d_ges, 3),
            "m2_pro_s": round(flaeche / t_ges, 3),
        })

    if not zeilen:
        raise SystemExit("Keine auswertbaren Karten gefunden.")
    fehlend = sorted(set(laeufe) - {z["name"] for z in zeilen})
    fehlend = [n for n in fehlend if n.startswith(a.praefix)]
    if fehlend:
        print(f"[WARNUNG] ohne Karte: {', '.join(fehlend)}\n")

    with a.out.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(zeilen[0]), delimiter=";")
        w.writeheader()
        w.writerows(zeilen)
    print(f"geschrieben: {a.out}  ({len(zeilen)} Laeufe)\n")

    # Zusammenfassung je Szene, t-Verteilung mit n = 10
    T95 = {5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228}
    for szene, label in (("sr", "Saeulenraum"), ("uni", "Ringflur")):
        g = [z for z in zeilen if z["szene"] == szene]
        if len(g) < 2:
            continue
        print(f"--- {label}  (n = {len(g)}) ---")
        for spalte, einheit in (("abdeckung_pct", "%"), ("m2_pro_m", "m^2/m"),
                                ("m2_pro_s", "m^2/s")):
            v = [z[spalte] for z in g]
            m, s = statistics.mean(v), statistics.stdev(v)
            t = T95.get(len(g) - 1, 2.262)
            h = t * s / math.sqrt(len(g))
            print(f"  {spalte:<14} {m:8.2f} +/- {s:5.2f} {einheit:<6} "
                  f"95%-KI [{m-h:.2f}; {m+h:.2f}]")
        print()


if __name__ == "__main__":
    main()

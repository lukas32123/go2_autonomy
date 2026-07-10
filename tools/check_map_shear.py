#!/usr/bin/env python3
import sys, math, statistics
with open(sys.argv[1], 'rb') as f:
    def tok():
        buf = b''
        while True:
            c = f.read(1)
            if not c: return buf
            if c == b'#': f.readline(); continue
            if c.isspace():
                if buf: return buf
                continue
            buf += c
    tok(); w = int(tok()); h = int(tok()); tok()
    data = f.read(w * h)
OCC = 50
left = lambda r: next((c for c in range(w) if data[r*w+c] < OCC), None)
top  = lambda c: next((r for r in range(h) if data[r*w+c] < OCC), None)
med  = lambda fn, idx: (lambda v: statistics.median(v) if v else None)(
                        [x for x in (fn(i) for i in idx) if x is not None])
rt, rb = range(int(h*.2), int(h*.3)), range(int(h*.7), int(h*.8))
ct, cb = range(int(w*.2), int(w*.3)), range(int(w*.7), int(w*.8))
lt, lb, tt, tb = med(left,rt), med(left,rb), med(top,ct), med(top,cb)
print(f"Karte: {w} x {h} px  (~{w*.05:.2f} x {h*.05:.2f} m; ideal 200x200)")
if None not in (lt,lb):
    print(f"Linke Wand: {lb-lt:+.0f} px ueber {rb.start-rt.start} px -> {math.degrees(math.atan2(lb-lt, rb.start-rt.start)):+.1f} Grad")
if None not in (tt,tb):
    print(f"Obere Wand: {tb-tt:+.0f} px ueber {cb.start-ct.start} px -> {math.degrees(math.atan2(tb-tt, cb.start-ct.start)):+.1f} Grad")
print("Ideal ~0 Grad; mehrere Grad = Scherung.")

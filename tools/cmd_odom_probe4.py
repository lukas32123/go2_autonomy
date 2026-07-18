#!/usr/bin/env python3
"""cmd_odom_probe4.py -- schlanke 4-Ebenen-Dreh-Haenger-Diagnose (token-sparsam).

Misst dieselben vier Ebenen wie zuvor
    controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed
                      -> collision_monitor -> /cmd_vel -> /odom (ist),
druckt aber im Default NICHT jeden Tick (das erschlaegt jedes Log), sondern nur:
  - Haenger-BEGINN und -ENDE (mit Dauer),
  - einen Herzschlag alle 2 s,
  - beim Beenden (Strg-C) eine kompakte ZUSAMMENFASSUNG inkl. Totbereich-Tabelle
    (Befehl out_wz  ->  Ist-Drehung ist_wz), die den Impedanz-Bruch in ~6 Zeilen belegt.

Den vollen Strom pro Tick gibt es weiter mit  -p verbose:=true  (fuer Detailanalyse).

Start (im Container, waehrend der Stack laeuft, im 6. Terminal):
    python3 /root/repo/tools/cmd_odom_probe4.py --ros-args -p use_sim_time:=true

Optional:
    -p verbose:=true      voller Strom pro Tick (wie die erste Version)
    -p rate_hz:=20.0      Abtast-/Bewertungstakt [Hz]
    -p heartbeat_s:=2.0   Abstand der Herzschlag-Zeilen [s]
    -p cmd_on:=0.05       ab hier gilt "es wird etwas befohlen"
    -p ist_off:=0.03      darunter gilt "der Roboter tut nichts"

Bedienung: Stack starten, Weststart abwarten, Probe starten, ~15-20 s laufen lassen,
Strg-C. Dann NUR den Zusammenfassungs-Block als Text pasten.
"""

# --- Reine, ROS-freie Logik (unit-testbar ohne rclpy) --------------------------

def classify(nav_vx, nav_wz, out_vx, out_wz, ist_vx, ist_wz, cmd_on, ist_off):
    """(is_hang, note). is_hang: out befiehlt etwas, /odom regt sich nicht.
    note: 'AUSF-LUECKE' (out>0, ist~0 -> Policy/Ausfuehrung) |
          'KETTE-GEDROSSELT' (nav>0, out~0 -> smoother/collision_monitor) | ''."""
    out_active = abs(out_vx) > cmd_on or abs(out_wz) > cmd_on
    moves = abs(ist_vx) > ist_off or abs(ist_wz) > ist_off
    nav_active = abs(nav_vx) > cmd_on or abs(nav_wz) > cmd_on
    out_dead = abs(out_vx) <= cmd_on and abs(out_wz) <= cmd_on
    is_hang = out_active and not moves
    if is_hang:
        note = "AUSF-LUECKE"
    elif nav_active and out_dead:
        note = "KETTE-GEDROSSELT"
    else:
        note = ""
    return is_hang, note


WZ_EDGES = [0.10, 0.20, 0.30]        # Bin-Grenzen fuer |out_wz|
WZ_LABELS = ["0.00-0.10", "0.10-0.20", "0.20-0.30", "0.30+"]


def wz_bin(absw):
    for i, e in enumerate(WZ_EDGES):
        if absw < e:
            return i
    return len(WZ_EDGES)


# --- ROS-Knoten (rclpy erst in main -> Modul bleibt fuer Tests importierbar) ---

def main():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    class Probe(Node):
        def __init__(self):
            super().__init__("cmd_odom_probe4")
            self.declare_parameter("verbose", False)
            self.declare_parameter("rate_hz", 20.0)
            self.declare_parameter("heartbeat_s", 2.0)
            self.declare_parameter("cmd_on", 0.05)
            self.declare_parameter("ist_off", 0.03)
            self.verbose = bool(self.get_parameter("verbose").value)
            self.rate_hz = float(self.get_parameter("rate_hz").value)
            self.heartbeat_s = float(self.get_parameter("heartbeat_s").value)
            self.cmd_on = float(self.get_parameter("cmd_on").value)
            self.ist_off = float(self.get_parameter("ist_off").value)

            self.nav_vx = self.nav_wz = 0.0
            self.smo_vx = self.smo_wz = 0.0
            self.out_vx = self.out_wz = 0.0
            self.ist_vx = self.ist_wz = 0.0
            self.t0 = None
            self.last_beat = None
            # Haenger-Verfolgung
            self.in_hang = False
            self.hang_start = None
            self.hang_events = []        # [(start, dauer)]
            # Summen fuer Zusammenfassung
            self.ticks = 0
            self.max_out_vx = 0.0
            self.max_ist_vx = 0.0
            self.max_chain_gap = 0.0     # max |nav_wz - out_wz| (grobe Ketten-Kontrolle)
            self.chain_gap_ticks = 0     # Ticks mit |nav_wz-out_wz| > cmd_on
            nb = len(WZ_EDGES) + 1
            self.bin_n = [0] * nb
            self.bin_ist = [0.0] * nb
            self.bin_out = [0.0] * nb
            # Oszillations-Metrik: Vorzeichenwechsel des Befehls
            self.last_sign = 0
            self.reversals = 0
            self.first_rev_t = None
            self.last_rev_t = None
            self.ticks_fullspeed = 0      # |out_wz| >= 1.0 (Shim-Vollgas)

            self.create_subscription(Twist, "/cmd_vel_nav", self._nav, 10)
            self.create_subscription(Twist, "/cmd_vel_smoothed", self._smo, 10)
            self.create_subscription(Twist, "/cmd_vel", self._out, 10)
            self.create_subscription(Odometry, "/odom", self._odom, 20)
            self.create_timer(1.0 / self.rate_hz, self._tick)

            mode = "VERBOSE (jeder Tick)" if self.verbose else "schlank (Haenger + Herzschlag + Summary)"
            self.get_logger().info(
                f"Probe4 aktiv @ {self.rate_hz:.0f} Hz, Modus: {mode}. "
                f"cmd_on={self.cmd_on}, ist_off={self.ist_off}. Strg-C -> Zusammenfassung."
            )

        def _nav(self, m):
            self.nav_vx, self.nav_wz = m.linear.x, m.angular.z

        def _smo(self, m):
            self.smo_vx, self.smo_wz = m.linear.x, m.angular.z

        def _out(self, m):
            self.out_vx, self.out_wz = m.linear.x, m.angular.z

        def _odom(self, m):
            self.ist_vx = m.twist.twist.linear.x
            self.ist_wz = m.twist.twist.angular.z

        def _tick(self):
            t = self.get_clock().now().nanoseconds * 1e-9
            if self.t0 is None:
                self.t0 = t
                self.last_beat = t
            ts = t - self.t0
            self.ticks += 1

            # Summen
            self.max_out_vx = max(self.max_out_vx, abs(self.out_vx))
            self.max_ist_vx = max(self.max_ist_vx, abs(self.ist_vx))
            gap = abs(self.nav_wz - self.out_wz)
            self.max_chain_gap = max(self.max_chain_gap, gap)
            if gap > self.cmd_on:
                self.chain_gap_ticks += 1
            b = wz_bin(abs(self.out_wz))
            self.bin_n[b] += 1
            # WICHTIG: gleichgerichtet mitteln, sonst loeschen sich Links-/Rechtsdrehungen aus
            sgn = 1.0 if self.out_wz >= 0.0 else -1.0
            self.bin_ist[b] += sgn * self.ist_wz
            self.bin_out[b] += abs(self.out_wz)
            if abs(self.out_wz) >= 1.0:
                self.ticks_fullspeed += 1
            # Richtungswechsel des Befehls zaehlen (nur oberhalb der Rauschschwelle)
            if abs(self.out_wz) > self.cmd_on:
                s_now = 1 if self.out_wz > 0 else -1
                if self.last_sign != 0 and s_now != self.last_sign:
                    self.reversals += 1
                    if self.first_rev_t is None:
                        self.first_rev_t = ts
                    self.last_rev_t = ts
                self.last_sign = s_now

            is_hang, note = classify(
                self.nav_vx, self.nav_wz, self.out_vx, self.out_wz,
                self.ist_vx, self.ist_wz, self.cmd_on, self.ist_off,
            )

            if self.verbose:
                flag = ""
                if is_hang:
                    flag = f"  <== HAENGER [{note}]" if note else "  <== HAENGER"
                elif note:
                    flag = f"  ({note})"
                print(f"t={ts:8.2f} | nav wz={self.nav_wz:+6.3f} | out wz={self.out_wz:+6.3f} "
                      f"| ist wz={self.ist_wz:+6.3f} | ist vx={self.ist_vx:+6.3f}{flag}", flush=True)

            # Haenger-Kanten (immer, auch im schlanken Modus)
            if is_hang and not self.in_hang:
                self.in_hang = True
                self.hang_start = ts
                if not self.verbose:
                    print(f"[HAENGER-Beginn] t={ts:7.2f}s  out_wz={self.out_wz:+.3f}  "
                          f"ist_wz={self.ist_wz:+.3f}", flush=True)
            elif not is_hang and self.in_hang:
                self.in_hang = False
                dur = ts - self.hang_start
                self.hang_events.append((self.hang_start, dur))
                if not self.verbose and dur >= 0.15:
                    print(f"[HAENGER-Ende]   t={ts:7.2f}s  Dauer={dur:4.1f}s", flush=True)

            # Herzschlag
            if not self.verbose and (t - self.last_beat) >= self.heartbeat_s:
                self.last_beat = t
                ngesamt = len(self.hang_events) + (1 if self.in_hang else 0)
                sgesamt = sum(d for _, d in self.hang_events)
                print(f"  t={ts:7.2f}s | out_wz={self.out_wz:+.3f} ist_wz={self.ist_wz:+.3f} "
                      f"| Haenger bisher: {ngesamt} ({sgesamt:.1f}s)", flush=True)

        def print_summary(self):
            ts_end = 0.0 if self.t0 is None else (self.get_clock().now().nanoseconds * 1e-9 - self.t0)
            if self.in_hang and self.hang_start is not None:
                self.hang_events.append((self.hang_start, ts_end - self.hang_start))
            kurz = [d for _, d in self.hang_events if d < 0.5]
            lang = [d for _, d in self.hang_events if d >= 0.5]
            s_lang = sum(lang)
            pct = (100.0 * s_lang / ts_end) if ts_end > 0 else 0.0
            rev_rate = (self.reversals / ts_end) if ts_end > 0 else 0.0
            fs_pct = (100.0 * self.ticks_fullspeed / self.ticks) if self.ticks else 0.0

            P = lambda x: print(x, flush=True)
            P("=" * 64)
            P("[SUMMARY] Dreh-Diagnose")
            P("=" * 64)
            P(f"[SUMMARY] Laufzeit (sim)       : {ts_end:.1f} s  ({self.ticks} Ticks)")
            P(f"[SUMMARY] ECHTE Haenger (>0.5s): {len(lang)}  | gesamt {s_lang:.1f} s ({pct:.0f}%)"
              f" | laengster {max(lang, default=0.0):.1f} s")
            P(f"[SUMMARY] Wendepunkte (<0.5s)  : {len(kurz)}   (Nulldurchgang beim Richtungswechsel,"
              f" kein Haenger)")
            P(f"[SUMMARY] OSZILLATION: Richtungswechsel des Befehls = {self.reversals}"
              f"  ({rev_rate:.2f}/s, alle {1.0/rev_rate:.1f} s)" if rev_rate > 0 else
              f"[SUMMARY] OSZILLATION: Richtungswechsel des Befehls = {self.reversals}")
            P(f"[SUMMARY] Zeit mit |out_wz|>=1.0: {fs_pct:.0f} %   (Shim-Vollgas)")
            P(f"[SUMMARY] max |out vx|         : {self.max_out_vx:.3f} m/s   (0 = nie vorwaerts befohlen)")
            P(f"[SUMMARY] max |ist vx|         : {self.max_ist_vx:.3f} m/s")
            P(f"[SUMMARY] Kette max|nav-out|wz : {self.max_chain_gap:.3f}  in {self.chain_gap_ticks}/{self.ticks} Ticks")
            P("[SUMMARY] Tracking (gleichgerichtet gemittelt, Vorzeichen bereinigt):")
            P(f"[SUMMARY]   {'bin':11s} {'n':>5s} {'mean |out|':>11s} {'mean ist':>10s} {'Track':>7s}")
            for i in range(len(WZ_EDGES) + 1):
                nb = self.bin_n[i]
                if nb == 0:
                    P(f"[SUMMARY]   {WZ_LABELS[i]:11s} {0:5d} {'n/a':>11s} {'n/a':>10s} {'n/a':>7s}")
                    continue
                mo = self.bin_out[i] / nb
                mi = self.bin_ist[i] / nb
                tr = (mi / mo) if mo > 1e-3 else float('nan')
                P(f"[SUMMARY]   {WZ_LABELS[i]:11s} {nb:5d} {mo:+11.3f} {mi:+10.3f} {tr:7.2f}")
            P("=" * 64)
            P("[SUMMARY] CSV;laufzeit;ticks;haenger_n;haenger_s;wenden;reversals;rev_pro_s;fullspeed_pct;max_out_vx")
            P(f"[SUMMARY] CSV;{ts_end:.1f};{self.ticks};{len(lang)};{s_lang:.1f};{len(kurz)};"
              f"{self.reversals};{rev_rate:.2f};{fs_pct:.0f};{self.max_out_vx:.3f}")
            P("=" * 64)

    rclpy.init()
    n = Probe()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.print_summary()
        try:
            n.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

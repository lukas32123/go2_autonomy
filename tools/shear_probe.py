#!/usr/bin/env python3
"""shear_probe.py -- entscheidet, WORAN die map->odom-Yaw-Drift haengt (Shear-Ursache).

Hintergrund: Bei Ground-Truth-Odometrie MUSS map->odom konstant sein. Gemessen wandert
es um 5-8 deg pro Lauf; genau diese Wanderung schert die Karte. Offen ist, WANN die
Wanderung entsteht. Zwei Hypothesen:

  H_KIPP  Sensorkippung: der Fehler entsteht, wenn der Rumpf stark nickt/rollt,
          weil base_link->laser statisch waagerecht angenommen wird.
  H_LAG   Zeitversatz: der Scan gehoert zu einem anderen Zeitpunkt als die Pose,
          mit der er gestempelt wird. Dann ist der Winkelfehler ~ omega * dt,
          d.h. der Fehler entsteht beim DREHEN, unabhaengig von der Kippung.

Diese Probe ist rein passiv (nur Subscriber), startet nichts und veraendert nichts.
Sie erkennt jeden Sprung in map->odom-Yaw und notiert, was der Roboter im Moment
davor getan hat (Drehrate und Neigung). Am Ende:
  - Anteil der Gesamtdrift, der bei schneller Drehung entstand   -> spricht fuer H_LAG
  - Anteil, der bei starker Kippung entstand                     -> spricht fuer H_KIPP
  - Terzil-Vergleich: was unterscheidet grosse von kleinen Spruengen?

Start (im Container, parallel zu eval_probe, waehrend ein Lauf laeuft):
    python3 /root/repo/tools/shear_probe.py <name> --ros-args -p use_sim_time:=true
Am Ende (nach HOME ERREICHT) Strg-C -> kompakter Block + CSV-Zeile.

Optional:
    -p jump_min_deg:=0.02   ab welchem Yaw-Sprung gezaehlt wird
    -p lookback_s:=0.40     Fenster vor dem Sprung, in dem omega/Neigung gemessen werden
    -p omega_fast:=0.30     Schwelle "dreht schnell" [rad/s]
    -p tilt_high_deg:=2.0   Schwelle "kippt stark" [deg]
"""

import math


# --- ROS-freie Mathematik (offline testbar) -----------------------------------

def quat_to_rpy(x, y, z, w):
    """Quaternion -> (roll, pitch, yaw) in rad."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def wrap_pi(a):
    """Winkeldifferenz auf (-pi, pi] normieren."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def pearson(a, b):
    n = len(a)
    if n < 3:
        return float('nan')
    ma, mb = sum(a) / n, sum(b) / n
    num = sum((x - ma) * (y - mb) for x, y in zip(a, b))
    den = (sum((x - ma) ** 2 for x in a) * sum((y - mb) ** 2 for y in b)) ** 0.5
    return num / den if den > 1e-12 else float('nan')


def terciles(jumps, key):
    """jumps: Liste von dicts. Gibt (unteres Drittel, oberes Drittel) nach 'dyaw' zurueck."""
    s = sorted(jumps, key=lambda j: j["dyaw"])
    k = max(1, len(s) // 3)
    return s[:k], s[-k:]


# --- ROS-Knoten ---------------------------------------------------------------

def main():
    import sys
    import rclpy
    from rclpy.node import Node
    from tf2_msgs.msg import TFMessage
    from nav_msgs.msg import Odometry

    name = "lauf"
    argv = [a for a in sys.argv[1:] if not a.startswith("-")]
    if argv:
        name = argv[0]

    class ShearProbe(Node):
        def __init__(self):
            super().__init__("shear_probe")
            self.declare_parameter("jump_min_deg", 0.02)
            self.declare_parameter("lookback_s", 0.40)
            self.declare_parameter("omega_fast", 0.30)
            self.declare_parameter("tilt_high_deg", 2.0)
            self.jump_min = float(self.get_parameter("jump_min_deg").value)
            self.lookback = float(self.get_parameter("lookback_s").value)
            self.omega_fast = float(self.get_parameter("omega_fast").value)
            self.tilt_high = float(self.get_parameter("tilt_high_deg").value)

            self.hist = []          # (t, omega_signed, roll_deg, pitch_deg)
            self.roll_sum = self.pitch_sum = 0.0
            self.n_move = 0
            self.last_yaw = None
            self.jumps = []
            self.t0 = None
            self.t_last = 0.0

            self.create_subscription(TFMessage, "/tf", self._tf, 50)
            self.create_subscription(Odometry, "/odom", self._odom, 50)
            self.get_logger().info(
                f"shear_probe aktiv (Lauf '{name}'). Passiv. Am Ende Strg-C.")

        def _now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def _odom(self, m):
            t = self._now()
            if self.t0 is None:
                self.t0 = t
            q = m.pose.pose.orientation
            roll, pitch, _ = quat_to_rpy(q.x, q.y, q.z, q.w)
            rd, pd = math.degrees(roll), math.degrees(pitch)
            w = m.twist.twist.angular.z                      # VORZEICHEN behalten
            self.hist.append((t, w, rd, pd))
            if abs(m.twist.twist.linear.x) > 0.05 or abs(w) > 0.05:
                self.roll_sum += rd; self.pitch_sum += pd; self.n_move += 1
            cut = t - 2.0
            if len(self.hist) > 400:
                self.hist = [h for h in self.hist if h[0] >= cut]
            self.t_last = t - self.t0

        def _tf(self, msg):
            for tr in msg.transforms:
                if tr.header.frame_id != "map" or tr.child_frame_id != "odom":
                    continue
                t = self._now()
                if self.t0 is None:
                    self.t0 = t
                q = tr.transform.rotation
                _, _, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
                if self.last_yaw is None:
                    self.last_yaw = yaw
                    return
                d_signed = math.degrees(wrap_pi(yaw - self.last_yaw))
                self.last_yaw = yaw
                if abs(d_signed) < self.jump_min:
                    return
                lo = t - self.lookback
                win = [h for h in self.hist if lo <= h[0] <= t]
                if not win:
                    return
                k = len(win)
                self.jumps.append({
                    "t": t - self.t0,
                    "dyaw": abs(d_signed),
                    "dyaw_s": d_signed,
                    "omega": max(abs(h[1]) for h in win),
                    "omega_s": sum(h[1] for h in win) / k,
                    "tilt": max(max(abs(h[2]), abs(h[3])) for h in win),
                    "roll_s": sum(h[2] for h in win) / k,
                    "pitch_s": sum(h[3] for h in win) / k,
                })

        def report(self):
            P = lambda s: print(s, flush=True)
            J = self.jumps
            P("=" * 66)
            P(f"=== SHEAR-PROBE: Lauf '{name}' ===")
            P("=" * 66)
            if len(J) < 6:
                P(f"Zu wenige Spruenge erfasst ({len(J)}). Laenger laufen lassen "
                  f"oder jump_min_deg senken.")
                P("=" * 66)
                return
            tot = sum(j["dyaw"] for j in J)
            P(f"Laufzeit (sim)          : {self.t_last:.1f} s")
            P(f"map->odom Yaw-Spruenge  : {len(J)} | Summe |dyaw| = {tot:.2f} deg "
              f"| groesster {max(j['dyaw'] for j in J):.2f} deg")
            P("")
            P("--- WO entsteht die Drift? (Anteil an der Gesamt-Yaw-Drift) ---")
            fast = sum(j["dyaw"] for j in J if j["omega"] >= self.omega_fast)
            slow = tot - fast
            hi = sum(j["dyaw"] for j in J if j["tilt"] >= self.tilt_high)
            lo_ = tot - hi
            P(f"  waehrend DREHUNG  |w| >= {self.omega_fast:.2f} rad/s : "
              f"{100*fast/tot:5.1f} %   ({fast:.2f} deg)")
            P(f"  waehrend ruhig    |w| <  {self.omega_fast:.2f} rad/s : "
              f"{100*slow/tot:5.1f} %   ({slow:.2f} deg)")
            P(f"  waehrend KIPPUNG  tilt >= {self.tilt_high:.1f} deg   : "
              f"{100*hi/tot:5.1f} %   ({hi:.2f} deg)")
            P(f"  waehrend flach    tilt <  {self.tilt_high:.1f} deg   : "
              f"{100*lo_/tot:5.1f} %   ({lo_:.2f} deg)")
            P("")
            if self.n_move:
                P("--- Dauerhafter Neigungs-BIAS waehrend der Fahrt (Mittelwert, nicht Amplitude) ---")
                P(f"  Roll-Mittel : {self.roll_sum/self.n_move:+.2f} deg     "
                  f"Nick-Mittel : {self.pitch_sum/self.n_move:+.2f} deg   "
                  f"({self.n_move} Samples in Bewegung)")
                P("")
            netto = sum(j["dyaw_s"] for j in J)
            P("--- Folgt die Drift der DREHRICHTUNG? (Test auf Zeitversatz) ---")
            lft = sum(j["dyaw_s"] for j in J if j["omega_s"] > 0.05)
            rgt = sum(j["dyaw_s"] for j in J if j["omega_s"] < -0.05)
            P(f"  Netto-Yaw-Drift gesamt              : {netto:+.2f} deg")
            P(f"  davon waehrend LINKSdrehung (w>0)   : {lft:+.2f} deg")
            P(f"  davon waehrend RECHTSdrehung (w<0)  : {rgt:+.2f} deg")
            P("  -> gegenlaeufige Vorzeichen = Fehler folgt der Drehung (Zeitversatz)")
            P("")
            dy = [j["dyaw"] for j in J]
            P("--- Korrelation der Sprunghoehe ---")
            P(f"  mit Drehrate |w| : r = {pearson(dy,[j['omega'] for j in J]):+.3f}")
            P(f"  mit Neigung      : r = {pearson(dy,[j['tilt']  for j in J]):+.3f}")
            dys = [j["dyaw_s"] for j in J]
            P("  vorzeichenbehaftet (dyaw gegen ...):")
            P(f"     omega : r = {pearson(dys,[j['omega_s'] for j in J]):+.3f}"
              f"     Roll : r = {pearson(dys,[j['roll_s'] for j in J]):+.3f}"
              f"     Nick : r = {pearson(dys,[j['pitch_s'] for j in J]):+.3f}")
            P("")
            low, high = terciles(J, "dyaw")
            f = lambda g, k: sum(x[k] for x in g) / len(g)
            P("--- Terzil-Vergleich (kleinste vs. groesste Spruenge) ---")
            P(f"  {'':16s} {'kleinstes Drittel':>18s} {'groesstes Drittel':>18s}")
            P(f"  {'dyaw [deg]':16s} {f(low,'dyaw'):18.3f} {f(high,'dyaw'):18.3f}")
            P(f"  {'|omega| [rad/s]':16s} {f(low,'omega'):18.3f} {f(high,'omega'):18.3f}")
            P(f"  {'Neigung [deg]':16s} {f(low,'tilt'):18.3f} {f(high,'tilt'):18.3f}")
            P("")
            P("--- Deutung ---")
            P("  omega im groessten Drittel hoeher, Links/Rechts gegenlaeufig -> H_LAG")
            P("  Neigung im groessten Drittel hoeher, Roll-Bias deutlich != 0   -> H_KIPP")
            P("  beide aehnlich -> keine der beiden dominiert, weitersuchen")
            P("=" * 66)
            P("CSV;name;t;n_jumps;yaw_sum;yaw_netto;drift_links;drift_rechts;"
              "roll_bias;nick_bias;pct_dreh;pct_kipp;r_omega;r_tilt;om_lo;om_hi;tilt_lo;tilt_hi")
            P(f"CSV;{name};{self.t_last:.1f};{len(J)};{tot:.2f};{netto:.2f};{lft:.2f};{rgt:.2f};"
              f"{self.roll_sum/max(1,self.n_move):.2f};{self.pitch_sum/max(1,self.n_move):.2f};"
              f"{100*fast/tot:.1f};"
              f"{100*hi/tot:.1f};{pearson(dy,[j['omega'] for j in J]):.3f};"
              f"{pearson(dy,[j['tilt'] for j in J]):.3f};{f(low,'omega'):.3f};"
              f"{f(high,'omega'):.3f};{f(low,'tilt'):.3f};{f(high,'tilt'):.3f}")
            P("=" * 66)

    rclpy.init()
    n = ShearProbe()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.report()
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

#!/usr/bin/env python3
"""tilt_tf_probe.py -- misst die ZWEI Zahlen, die den Karten-Shear entscheiden.

MESSUNG 1 -- Rumpfneigung (Roll/Nick) in GRAD
    /odom liefert die Orientierung von base_link im odom-Frame. odom ist in Isaac
    schwerkraft-ausgerichtet, also sind Roll/Nick daraus die echte Rumpfneigung.

    Warum das zaehlt: Der 2D-LiDAR ist starres Kind von /World/Robot/base, der TF
    base_link->laser ist aber STATISCH mit Identitaets-Orientierung. SLAM haelt die
    Scanebene damit fuer immer waagerecht -- sie ist es nie. Nickt der Rumpf um
    theta nach unten, trifft der Strahl den Boden bei r = laser_z / sin(theta):

        laser_z = 0.12 m:   1 Grad -> 6.9 m | 2 Grad -> 3.4 m | 3 Grad -> 2.3 m

    Ein Strahl, der eine 6 m entfernte Wand messen soll, meldet dann Boden bei 2.3 m.

    ABTASTRATE: Der Trab liegt bei ~2-3 Hz. Fuer eine belastbare AMPLITUDE braucht es
    mindestens ~20 Hz, besser 50+. Dieser Knoten hoert auf JEDE /odom-Nachricht und
    meldet die tatsaechlich erreichte Rate -- ist sie zu niedrig, ist das Ergebnis
    nicht belastbar und wird ausdruecklich als solches markiert.

MESSUNG 2 -- map->odom Drift
    Die Odometrie ist Ground Truth aus der Simulation. Sie driftet nicht.
    -> map->odom MUSS eine Konstante sein.
    Jede Abweichung ist Scan-Matching, das eine bereits KORREKTE Pose "korrigiert".
    Das ist der Shear, live beim Entstehen.
    (Dieser Test steht seit dem 30.06. als [ZU VERIFIZIEREN] in der Architektur-Doku.)

Start (im Container, waehrend der Stack laeuft und der Roboter FAEHRT):
    python3 /root/repo/tools/tilt_tf_probe.py --ros-args -p use_sim_time:=true

Strg-C -> Abschlussbericht.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

LASER_Z = 0.12          # m -- Montagehoehe des LiDAR ueber base (aus play_go2_ros_scan.py)
REPORT_EVERY = 5.0      # s -- Zwischenbericht
MIN_RATE_HZ = 20.0      # darunter ist die Amplitudenmessung nicht belastbar


def quat_to_rpy(x, y, z, w):
    """Quaternion -> (roll, pitch, yaw) in rad."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))       # numerisch absichern
    pitch = math.asin(sinp)

    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class TiltProbe(Node):
    def __init__(self):
        super().__init__("tilt_tf_probe")

        # --- Messung 1: Neigung ---
        self.n_odom = 0
        self.t_first = None
        self.t_last = None
        self.roll_min = self.roll_max = None
        self.pitch_min = self.pitch_max = None
        self.roll_sq = 0.0
        self.pitch_sq = 0.0
        self.moving_samples = 0            # nur waehrend Bewegung -> Gangzyklus

        # --- Messung 2: map->odom ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.mo_first = None               # (tx, ty, yaw) beim ersten Fix
        self.mo_tx = []                    # Spannweiten
        self.mo_ty = []
        self.mo_yaw = []
        self.mo_fails = 0

        self.create_subscription(Odometry, "/odom", self.odom_cb, 50)
        self.create_timer(0.1, self.tf_tick)          # 10 Hz reicht fuer den Drift
        self.create_timer(REPORT_EVERY, self.report)

        self.get_logger().info(
            "Probe aktiv. Den Roboter FAHREN lassen (Explorer starten oder RViz-Ziel setzen)."
        )

    # ------------------------------------------------------------------
    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        roll, pitch, _ = quat_to_rpy(q.x, q.y, q.z, q.w)
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)

        t = self.get_clock().now().nanoseconds * 1e-9
        if self.t_first is None:
            self.t_first = t
        self.t_last = t
        self.n_odom += 1

        self.roll_min = roll if self.roll_min is None else min(self.roll_min, roll)
        self.roll_max = roll if self.roll_max is None else max(self.roll_max, roll)
        self.pitch_min = pitch if self.pitch_min is None else min(self.pitch_min, pitch)
        self.pitch_max = pitch if self.pitch_max is None else max(self.pitch_max, pitch)
        self.roll_sq += roll * roll
        self.pitch_sq += pitch * pitch

        v = msg.twist.twist
        if abs(v.linear.x) > 0.05 or abs(v.angular.z) > 0.05:
            self.moving_samples += 1

    # ------------------------------------------------------------------
    def tf_tick(self):
        try:
            tr = self.tf_buffer.lookup_transform("map", "odom", Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.mo_fails += 1
            return
        t = tr.transform.translation
        q = tr.transform.rotation
        _, _, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        yaw = math.degrees(yaw)
        if self.mo_first is None:
            self.mo_first = (t.x, t.y, yaw)
        self.mo_tx.append(t.x)
        self.mo_ty.append(t.y)
        self.mo_yaw.append(yaw)

    # ------------------------------------------------------------------
    def rate_hz(self):
        if self.t_first is None or self.t_last is None or self.t_last <= self.t_first:
            return 0.0
        return (self.n_odom - 1) / (self.t_last - self.t_first)

    def report(self):
        if self.n_odom < 10:
            self.get_logger().info("... noch zu wenige /odom-Nachrichten.")
            return
        hz = self.rate_hz()
        p_amp = (self.pitch_max - self.pitch_min) / 2.0
        r_amp = (self.roll_max - self.roll_min) / 2.0
        print(
            f"[{self.n_odom:6d} @ {hz:5.1f} Hz]  "
            f"Nick {self.pitch_min:+6.2f}..{self.pitch_max:+6.2f} deg (Ampl {p_amp:5.2f})  |  "
            f"Roll {self.roll_min:+6.2f}..{self.roll_max:+6.2f} deg (Ampl {r_amp:5.2f})",
            flush=True,
        )

    # ------------------------------------------------------------------
    def final(self):
        print("\n" + "=" * 72)
        print("ABSCHLUSSBERICHT")
        print("=" * 72)

        if self.n_odom < 10:
            print("Zu wenige /odom-Nachrichten -- kein Ergebnis.")
            return

        hz = self.rate_hz()
        p_amp = (self.pitch_max - self.pitch_min) / 2.0
        r_amp = (self.roll_max - self.roll_min) / 2.0
        p_rms = math.sqrt(self.pitch_sq / self.n_odom)
        r_rms = math.sqrt(self.roll_sq / self.n_odom)
        p_ext = max(abs(self.pitch_min), abs(self.pitch_max))

        print(f"\n--- MESSUNG 1: Rumpfneigung ({self.n_odom} Samples, {hz:.1f} Hz) ---")
        print(f"  davon in Bewegung : {self.moving_samples}")
        print(f"  Nick  : {self.pitch_min:+6.2f} .. {self.pitch_max:+6.2f} deg"
              f"   Amplitude {p_amp:5.2f} deg   RMS {p_rms:5.2f} deg")
        print(f"  Roll  : {self.roll_min:+6.2f} .. {self.roll_max:+6.2f} deg"
              f"   Amplitude {r_amp:5.2f} deg   RMS {r_rms:5.2f} deg")

        if hz < MIN_RATE_HZ:
            print(f"\n  [!] WARNUNG: {hz:.1f} Hz < {MIN_RATE_HZ:.0f} Hz.")
            print(f"      Der Trab liegt bei ~2-3 Hz. Bei dieser Rate ist die AMPLITUDE")
            print(f"      unterabgetastet und NICHT belastbar. Die Zahlen sind eine")
            print(f"      untere Schranke -- die echte Neigung ist groesser.")

        if p_ext > 0.05:
            r_hit = LASER_Z / math.sin(math.radians(p_ext))
            print(f"\n  --> Bei max. Nick {p_ext:.2f} deg trifft der Strahl den BODEN bei {r_hit:.1f} m.")
            print(f"      (LiDAR-Hoehe {LASER_Z} m; Raumdiagonale ~10 m)")
            if r_hit < 8.0:
                print(f"      BEFUND: Wandmessungen jenseits {r_hit:.1f} m werden durch")
                print(f"      Bodentreffer ersetzt. Die Nick-Hypothese ist BESTAETIGT.")
            else:
                print(f"      BEFUND: Bodentreffer erst jenseits des Raumes.")
                print(f"      Die Nick-Hypothese ist damit WIDERLEGT -- das Kippen erklaert")
                print(f"      den Shear NICHT. Verdaechtiger bleibt das Scan-Matching selbst.")
        else:
            print("\n  --> Neigung praktisch null. Nick-Hypothese WIDERLEGT.")

        print(f"\n--- MESSUNG 2: map->odom (Ground-Truth-Odometrie => MUSS konstant sein) ---")
        if not self.mo_tx:
            print(f"  Keine Transform bekommen ({self.mo_fails} Fehlversuche).")
            print("  Laeuft SLAM? Wurde der Explorer/ein Ziel gestartet?")
        else:
            dtx = max(self.mo_tx) - min(self.mo_tx)
            dty = max(self.mo_ty) - min(self.mo_ty)
            dyaw = max(self.mo_yaw) - min(self.mo_yaw)
            print(f"  Samples: {len(self.mo_tx)}")
            print(f"  tx  : Spanne {dtx*100:6.2f} cm")
            print(f"  ty  : Spanne {dty*100:6.2f} cm")
            print(f"  yaw : Spanne {dyaw:6.3f} deg")
            if dyaw > 0.5 or dtx > 0.05 or dty > 0.05:
                print(f"\n  BEFUND: map->odom ist NICHT konstant.")
                print(f"  Das Scan-Matching verschiebt eine bereits EXAKTE Pose.")
                print(f"  Das ist der Shear, live beim Entstehen.")
            else:
                print(f"\n  BEFUND: map->odom ist stabil. Das Scan-Matching haelt sich")
                print(f"  zurueck -- der Shear entsteht dann NICHT hier.")
        print("=" * 72 + "\n")


def main():
    rclpy.init(args=sys.argv)
    node = TiltProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.final()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

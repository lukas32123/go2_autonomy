#!/usr/bin/env python3
"""cmd_odom_probe.py -- loggt Befehl und Ist-Bewegung in EINER Zeile, sim-zeit-gestempelt.

Warum: /cmd_vel und /odom in zwei Terminals zu echoen laesst sich nicht korrelieren.
Dieser Knoten macht genau das moeglich -- und markiert Haenger automatisch.

Start (im Container, waehrend der Stack laeuft):
    python3 /root/repo/tools/cmd_odom_probe.py --ros-args -p use_sim_time:=true

Spalten:
    t_sim   Sim-Zeit seit Start [s]   <- fuer Kap. 5 die richtige Zeitbasis, NICHT Wall-Time
    cmd_vx / cmd_wz    was Nav2 final befiehlt (nach collision_monitor)
    ist_vx / ist_wz    was der Roboter tatsaechlich tut (/odom)
    Flag    HAENGER = es wird etwas befohlen, aber nichts passiert
            (|cmd| ueber Schwelle, |ist| darunter)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

CMD_ON = 0.05     # ab hier gilt "es wird etwas befohlen"
IST_OFF = 0.03    # darunter gilt "der Roboter tut nichts"


class Probe(Node):
    def __init__(self):
        super().__init__("cmd_odom_probe")
        self.cmd_vx = self.cmd_wz = 0.0
        self.ist_vx = self.ist_wz = 0.0
        self.t0 = None
        self.stall_since = None
        self.create_subscription(Twist, "/cmd_vel", self._cmd, 10)
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_timer(0.2, self._tick)          # 5 Hz
        self.get_logger().info(
            "Probe aktiv. Spalten: t_sim | cmd_vx cmd_wz | ist_vx ist_wz | Flag"
        )

    def _cmd(self, m):
        self.cmd_vx = m.linear.x
        self.cmd_wz = m.angular.z

    def _odom(self, m):
        self.ist_vx = m.twist.twist.linear.x
        self.ist_wz = m.twist.twist.angular.z

    def _tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t
        ts = t - self.t0

        befohlen = abs(self.cmd_vx) > CMD_ON or abs(self.cmd_wz) > CMD_ON
        passiert = abs(self.ist_vx) > IST_OFF or abs(self.ist_wz) > IST_OFF

        flag = ""
        if befohlen and not passiert:
            if self.stall_since is None:
                self.stall_since = ts
            flag = f"  <== HAENGER seit {ts - self.stall_since:5.1f}s"
        else:
            if self.stall_since is not None and (ts - self.stall_since) > 1.0:
                flag = f"  <== Haenger vorbei nach {ts - self.stall_since:.1f}s"
            self.stall_since = None

        print(
            f"t={ts:8.2f} | cmd vx={self.cmd_vx:+6.3f} wz={self.cmd_wz:+6.3f} "
            f"| ist vx={self.ist_vx:+6.3f} wz={self.ist_wz:+6.3f}{flag}",
            flush=True,
        )


def main():
    rclpy.init()
    n = Probe()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

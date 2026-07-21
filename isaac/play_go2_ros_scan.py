# play_go2_ros_scan.py
# ---------------------------------------------------------------------------
# Block 2 / Schritt 3b: Schritt 3a + 2D-LiDAR -> /scan.
#   Basis = verifiziertes play_go2_ros_tf.py (Schritt 3a).
#   NEU:
#     - 2D-RTX-LiDAR (Slamtec RPLIDAR_S2E, natives 2D-Profil) am base-Body,
#       Mount identisch zum statischen TF (0,0,0.12).
#     - FlatScan-Annotator (IsaacComputeRTXLidarFlatScan) -> sensor_msgs/LaserScan
#       auf /scan, sim-zeit-gestempelt, frame_id=laser, ueber denselben rclpy-Knoten.
#     - Einfacher 8x8-m-Raum (4 Waende), damit der Scan endliche Distanzen liefert.
#
#   Option 2 (bewusst): natives 2D-LiDAR statt 3D-XT32. Die gesamte Pipeline
#   (SLAM-Map -> Nav2-Costmaps -> Frontier) ist 2D; ein 3D-Sensor wuerde sofort
#   plattgedrueckt. /scan-Aufbau einheiten-robust (Auto-Grad->Radiant) + einmaliger
#   Diagnose-Ausdruck der Rohfelder, da FlatScan-Einheiten/Modus noch zu bestaetigen.
#
#   Frame-Konvention (REP-105): map -> odom -> base_link -> laser.
#     * map -> odom publiziert SPAETER SLAM Toolbox (Block 3), NICHT hier.
# ---------------------------------------------------------------------------

import argparse

from isaaclab.app import AppLauncher

# --- CLI / App-Start ---
parser = argparse.ArgumentParser(description="Go2 Flat-Policy mit /cmd_vel + /odom + /tf (ROS 2).")
parser.add_argument("--cmd_timeout", type=float, default=0.5, help="Watchdog [s]: ohne Twist -> Kommando 0.")
parser.add_argument("--cmd_topic", type=str, default="/cmd_vel", help="Twist-Topic.")
parser.add_argument("--laser_x", type=float, default=0.0, help="LiDAR-Mount x rel. base [m].")
parser.add_argument("--laser_y", type=float, default=0.0, help="LiDAR-Mount y rel. base [m].")
parser.add_argument("--laser_z", type=float, default=0.12, help="LiDAR-Mount z rel. base [m] (an realer Halterung messen).")
parser.add_argument(
    "--policy_path",
    type=str,
    default="/home/kilab/IsaacLab/logs/rsl_rl/unitree_go2_flat/2026-06-18_11-25-44/exported/policy.pt",
    help="Pfad zur exportierten TorchScript-Policy (policy.pt).",
)
parser.add_argument("--lidar_config", type=str, default="Slamtec_RPLIDAR_S2E",
                    help="2D-LiDAR-Profil (Asset-Name aus SUPPORTED_LIDAR_CONFIGS).")
parser.add_argument("--scan_topic", type=str, default="/scan", help="LaserScan-Topic.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- Bridge-Extension laden -> internes jazzy-rclpy ---
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# --- Imports nach App-Start ---
import math  # noqa: E402

import numpy as np  # noqa: E402
import torch  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, QoSDurabilityPolicy  # noqa: E402
from geometry_msgs.msg import Twist, TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402

from isaacsim.sensors.rtx import LidarRtx  # noqa: E402

try:
    from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG  # noqa: E402
except ImportError:
    from isaaclab_assets import UNITREE_GO2_CFG  # noqa: E402

# --- Konstanten aus env.yaml ---
SIM_DT = 0.005
DECIMATION = 4        # -> 50 Hz Policy
ACTION_SCALE = 0.25
FALL_Z = 0.15

# ROS-Frame-Namen (REP-105). USD-Body "base" -> "base_link".
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"
LASER_FRAME = "laser"

FLATSCAN = "IsaacComputeRTXLidarFlatScan"


def _stamp(t: float):
    sec = int(t)
    nanosec = int(round((t - sec) * 1e9))
    return sec, nanosec


class Go2RosBridge(Node):
    def __init__(self, cmd_topic: str, scan_topic: str, laser_offset):
        super().__init__("go2_ros_bridge")
        # --- Eingang: /cmd_vel ---
        self.vx = self.vy = self.wz = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.create_subscription(Twist, cmd_topic, self._cmd_cb, 10)
        # --- Ausgang: /clock /odom /tf /tf_static ---
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        static_qos = QoSProfile(depth=1)
        static_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL  # latched fuer spaete Joiner
        self.tf_static_pub = self.create_publisher(TFMessage, "/tf_static", static_qos)
        # --- Ausgang: /scan ---
        self.scan_pub = self.create_publisher(LaserScan, scan_topic, 10)
        self._scan_diag_done = False
        self.laser_offset = laser_offset
        self.get_logger().info(
            f"Bridge aktiv: sub {cmd_topic}; pub /clock /odom /tf ({ODOM_FRAME}->{BASE_FRAME}) "
            f"/tf_static ({BASE_FRAME}->{LASER_FRAME} @ {tuple(laser_offset)})"
        )

    def _cmd_cb(self, msg: Twist):
        self.vx = float(msg.linear.x)
        self.vy = float(msg.linear.y)
        self.wz = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def cmd_age_s(self) -> float:
        return (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9

    def publish_clock(self, t: float):
        m = Clock()
        m.clock.sec, m.clock.nanosec = _stamp(t)
        self.clock_pub.publish(m)

    def publish_odom_tf(self, t, pos, quat_wxyz, lin_b, ang_b):
        sec, nanosec = _stamp(t)
        qw, qx, qy, qz = quat_wxyz  # Isaac: (w,x,y,z) -> ROS: (x,y,z,w)
        # /odom
        od = Odometry()
        od.header.stamp.sec, od.header.stamp.nanosec = sec, nanosec
        od.header.frame_id = ODOM_FRAME
        od.child_frame_id = BASE_FRAME
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = pos
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = lin_b
        od.twist.twist.angular.x, od.twist.twist.angular.y, od.twist.twist.angular.z = ang_b
        self.odom_pub.publish(od)
        # /tf : odom -> base_link
        tf = TransformStamped()
        tf.header.stamp.sec, tf.header.stamp.nanosec = sec, nanosec
        tf.header.frame_id = ODOM_FRAME
        tf.child_frame_id = BASE_FRAME
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = pos
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_pub.publish(TFMessage(transforms=[tf]))

    def publish_static_tf(self, t: float):
        sec, nanosec = _stamp(t)
        ts = TransformStamped()
        ts.header.stamp.sec, ts.header.stamp.nanosec = sec, nanosec
        ts.header.frame_id = BASE_FRAME
        ts.child_frame_id = LASER_FRAME
        ts.transform.translation.x = self.laser_offset[0]
        ts.transform.translation.y = self.laser_offset[1]
        ts.transform.translation.z = self.laser_offset[2]
        ts.transform.rotation.x = 0.0
        ts.transform.rotation.y = 0.0
        ts.transform.rotation.z = 0.0
        ts.transform.rotation.w = 1.0
        self.tf_static_pub.publish(TFMessage(transforms=[ts]))

    def publish_scan(self, t: float, data) -> bool:
        """FlatScan-Annotatordaten -> sensor_msgs/LaserScan. True, wenn publiziert."""
        if not isinstance(data, dict):
            return False
        depth = np.asarray(data.get("linearDepthData", []), dtype=np.float32).ravel()
        if depth.size == 0:
            return False  # noch keine gueltigen Daten (Warmup)

        az = np.asarray(data.get("azimuthRange", [0.0, 0.0]), dtype=float).ravel()
        a_min, a_max = float(az[0]), float(az[1])
        dr = np.asarray(data.get("depthRange", [0.0, 0.0]), dtype=float).ravel()
        r_min = float(dr[0]) if dr.size >= 1 and dr[0] > 0 else 0.05
        r_max = float(dr[1]) if dr.size >= 2 and dr[1] > 0 else 100.0

        # Einheiten-robust: sieht die Spanne nach Grad aus (> 2*pi), in Radiant umrechnen.
        if abs(a_max - a_min) > 7.0:
            a_min, a_max = math.radians(a_min), math.radians(a_max)
        n = int(depth.size)
        inc = (a_max - a_min) / n if n > 0 else 0.0  # konsistent zur tatsaechlichen Strahlzahl

        if not self._scan_diag_done:
            self._scan_diag_done = True
            finite = depth[np.isfinite(depth) & (depth > 0) & (depth < 1e6)]
            self.get_logger().info(
                f"[SCAN-DIAG] N={n} angle=[{a_min:.4f},{a_max:.4f}] rad inc={inc:.6f} "
                f"horizRes_raw={data.get('horizontalResolution')} rangeM=[{r_min:.3f},{r_max:.3f}] "
                f"finite={finite.size}/{n} sample={np.round(finite[:6], 3).tolist() if finite.size else '[]'}"
            )

        ranges = depth.copy()
        ranges[~np.isfinite(ranges)] = float("inf")
        ranges[ranges <= 0.0] = float("inf")  # 0 = kein Treffer -> von SLAM ignoriert

        sec, nanosec = _stamp(t)
        msg = LaserScan()
        msg.header.stamp.sec, msg.header.stamp.nanosec = sec, nanosec
        msg.header.frame_id = LASER_FRAME
        msg.angle_min = a_min
        msg.angle_max = a_min + (len(ranges) - 1) * inc
        msg.angle_increment = inc
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = max(r_min, 0.0)
        msg.range_max = r_max
        msg.ranges = ranges.tolist()
        inten = np.asarray(data.get("intensitiesData", []), dtype=np.float32).ravel()
        if inten.size == n:
            msg.intensities = inten.tolist()
        self.scan_pub.publish(msg)
        return True


def design_scene() -> Articulation:
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/ground", ground_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=750.0, color=(1.0, 1.0, 1.0))
    light_cfg.func("/World/Light", light_cfg)

    # ---------------- Uni-Ringflur (parametrisch) ----------------
    # Grundriss nach realem Uni-Gebaeude: Ring um einen soliden Kern, zwei
    # gegenueberliegende Fluren breit, zwei schmal, Tuernischen als Merkmale.
    OUTER         = 30.0    # Aussenmass, Mittellinie der Nischen-Rueckwand [m]
    CORR_WIDE     = 6.0     # Flurbreite Nord/Sued [m]
    CORR_NARROW   = 3.0     # Flurbreite Ost/West [m]  (Vorgabe 2.5-4.0)
    NICHE_DEPTH   = 0.40    # Nischentiefe [m]
    NICHE_WIDTH   = 1.40    # Nischenbreite (Tuerbreite + Rahmen) [m]
    PITCH_WIDE    = 6.0     # Nischenabstand auf den breiten Seiten [m]
    PITCH_NARROW  = 4.0     # Nischenabstand auf den schmalen Seiten [m]
    CORNER_MARGIN = 1.5     # keine Nische naeher als das an einer Ecke [m]
    WALL_T        = 0.20    # Wanddicke [m]
    WALL_H        = 1.00    # Wandhoehe [m]

    half = OUTER / 2.0
    back = half - WALL_T / 2.0            # Innenflaeche der Rueckwand (Nischengrund)
    pier = back - NICHE_DEPTH             # Flurgrenze aussen (Pfeilerflaeche)
    core_y = pier - CORR_WIDE             # Kernflaeche Nord/Sued
    core_x = pier - CORR_NARROW           # Kernflaeche Ost/West (Pfeilerflaeche)
    core_solid = core_x - NICHE_DEPTH     # Kern-Vollkoerper reicht bis hier
    z = WALL_H / 2.0

    gray = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.60, 0.60, 0.60))
    dark = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.45, 0.45, 0.45))
    coll = sim_utils.CollisionPropertiesCfg()

    def niche_centers(length, pitch):
        """Nischenmitten, symmetrisch um 0, mit Abstand zu den Ecken."""
        lim = length / 2.0 - CORNER_MARGIN - NICHE_WIDTH / 2.0
        out, k = [], 0
        while k * pitch <= lim:
            out.extend([0.0] if k == 0 else [-k * pitch, k * pitch])
            k += 1
        return sorted(out)

    def pier_spans(length, centers):
        """Intervalle zwischen den Nischen -> dort stehen die Pfeiler."""
        out, lo = [], -length / 2.0
        for c in centers:
            hi = c - NICHE_WIDTH / 2.0
            if hi - lo > 0.05:
                out.append((lo, hi))
            lo = c + NICHE_WIDTH / 2.0
        if length / 2.0 - lo > 0.05:
            out.append((lo, length / 2.0))
        return out

    n = 0
    for s in (1.0, -1.0):
        # --- breite Seiten (Nord/Sued): Rueckwand + Pfeiler, Nischen nur aussen
        w = sim_utils.CuboidCfg(size=(OUTER, WALL_T, WALL_H),
                                visual_material=gray, collision_props=coll)
        w.func("/World/wall_ns_%d" % n, w, translation=(0.0, s * half, z))
        n += 1
        for a, b in pier_spans(OUTER, niche_centers(OUTER, PITCH_WIDE)):
            p = sim_utils.CuboidCfg(size=(b - a, NICHE_DEPTH, WALL_H),
                                    visual_material=gray, collision_props=coll)
            p.func("/World/pier_ns_%d" % n, p,
                   translation=((a + b) / 2.0, s * (pier + NICHE_DEPTH / 2.0), z))
            n += 1
        # --- schmale Seiten (Ost/West): Rueckwand + Pfeiler
        w = sim_utils.CuboidCfg(size=(WALL_T, OUTER, WALL_H),
                                visual_material=gray, collision_props=coll)
        w.func("/World/wall_ew_%d" % n, w, translation=(s * half, 0.0, z))
        n += 1
        for a, b in pier_spans(OUTER, niche_centers(OUTER, PITCH_NARROW)):
            p = sim_utils.CuboidCfg(size=(NICHE_DEPTH, b - a, WALL_H),
                                    visual_material=gray, collision_props=coll)
            p.func("/World/pier_ew_%d" % n, p,
                   translation=(s * (pier + NICHE_DEPTH / 2.0), (a + b) / 2.0, z))
            n += 1

    # --- Kern: Vollkoerper, dazu Pfeiler auf den Ost/West-Flaechen ---------
    core = sim_utils.CuboidCfg(size=(2.0 * core_solid, 2.0 * core_y, WALL_H),
                               visual_material=dark, collision_props=coll)
    core.func("/World/core", core, translation=(0.0, 0.0, z))
    n += 1
    core_len = 2.0 * core_y
    for s in (1.0, -1.0):
        for a, b in pier_spans(core_len, niche_centers(core_len, PITCH_NARROW)):
            p = sim_utils.CuboidCfg(size=(NICHE_DEPTH, b - a, WALL_H),
                                    visual_material=dark, collision_props=coll)
            p.func("/World/core_pier_%d" % n, p,
                   translation=(s * (core_solid + NICHE_DEPTH / 2.0), (a + b) / 2.0, z))
            n += 1

    print("[SZENE] Uni-Ringflur %.1f m | Flur N/S %.1f m, O/W %.1f m | "
          "Nische %.2f x %.2f m | %d Prims"
          % (OUTER, pier - core_y, pier - core_x, NICHE_WIDTH, NICHE_DEPTH, n))

    robot_cfg = UNITREE_GO2_CFG.replace(prim_path="/World/Robot")
    # Startpose in der Mitte des SUEDflurs (breite Seite), Blick +x (Ost).
    start_y = -(core_y + CORR_WIDE / 2.0)
    robot_cfg.init_state.pos = (0.0, start_y, 0.4)
    return Articulation(robot_cfg)


def compute_obs(robot, command, last_action):
    return torch.cat(
        [
            robot.data.root_lin_vel_b,
            robot.data.root_ang_vel_b,
            robot.data.projected_gravity_b,
            command,
            robot.data.joint_pos - robot.data.default_joint_pos,
            robot.data.joint_vel - robot.data.default_joint_vel,
            last_action,
        ],
        dim=-1,
    )


def main():
    sim = SimulationContext(SimulationCfg(dt=SIM_DT, render_interval=DECIMATION, device=args_cli.device))
    robot = design_scene()

    # 2D-LiDAR als Kind des base-Bodies -> bewegt sich starr mit dem Roboter.
    # Mount identisch zum statischen TF base_link->laser.
    lidar = LidarRtx(
        prim_path="/World/Robot/base/lidar",
        name="go2_lidar",
        translation=np.array([args_cli.laser_x, args_cli.laser_y, args_cli.laser_z]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),  # w,x,y,z (identitaet)
        config_file_name=args_cli.lidar_config,
    )
    lidar.attach_annotator(FLATSCAN)
    print(f"[INFO] LiDAR '{args_cli.lidar_config}' erstellt, FlatScan angehaengt, "
          f"render_product={lidar.get_render_product_path()}")

    sim.reset()
    device = sim.device

    policy = torch.jit.load(args_cli.policy_path, map_location=device)
    policy.eval()
    print(f"[INFO] Policy geladen: {args_cli.policy_path}")

    rclpy.init()
    laser_offset = (args_cli.laser_x, args_cli.laser_y, args_cli.laser_z)
    node = Go2RosBridge(args_cli.cmd_topic, args_cli.scan_topic, laser_offset)

    command = torch.zeros((1, 3), device=device, dtype=torch.float32)

    def reset_robot():
        root_state = robot.data.default_root_state.clone()
        robot.write_root_pose_to_sim(root_state[:, :7])
        robot.write_root_velocity_to_sim(root_state[:, 7:])
        robot.write_joint_state_to_sim(robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone())
        robot.reset()

    reset_robot()
    last_action = torch.zeros((1, 12), device=device, dtype=torch.float32)

    sim_time = 0.0
    node.publish_static_tf(sim_time)  # einmal sofort (latched)
    step = 0

    while simulation_app.is_running():
        # --- /cmd_vel uebernehmen (+ Watchdog) ---
        rclpy.spin_once(node, timeout_sec=0.0)
        if node.cmd_age_s() > args_cli.cmd_timeout:
            command[0, 0] = command[0, 1] = command[0, 2] = 0.0
        else:
            command[0, 0] = node.vx
            command[0, 1] = node.vy
            command[0, 2] = node.wz

        # --- Policy-Schritt ---
        obs = compute_obs(robot, command, last_action)
        with torch.inference_mode():
            raw_action = policy(obs)
        targets = robot.data.default_joint_pos + ACTION_SCALE * raw_action
        last_action = raw_action.clone()

        # --- Physik-Substeps (Sim-Zeit mitfuehren) ---
        robot.set_joint_position_target(targets)
        for _ in range(DECIMATION):
            robot.write_data_to_sim()
            sim.step(render=False)
            robot.update(SIM_DT)
            sim_time += SIM_DT
        sim.render()

        # --- ROS-Ausgabe (Ground-Truth-Pose) ---
        pos = robot.data.root_pos_w[0]
        quat = robot.data.root_quat_w[0]      # (w,x,y,z)
        lin_b = robot.data.root_lin_vel_b[0]
        ang_b = robot.data.root_ang_vel_b[0]
        node.publish_clock(sim_time)
        node.publish_odom_tf(
            sim_time,
            (float(pos[0]), float(pos[1]), float(pos[2])),
            (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
            (float(lin_b[0]), float(lin_b[1]), float(lin_b[2])),
            (float(ang_b[0]), float(ang_b[1]), float(ang_b[2])),
        )
        if step % 50 == 0:  # static TF ~1x/s nachsenden (robust)
            node.publish_static_tf(sim_time)

        # --- /scan aus FlatScan-Annotator (direkt vom Annotator gelesen) ---
        annots = lidar.get_annotators()
        if FLATSCAN in annots:
            try:
                scan_data = annots[FLATSCAN].get_data()
            except Exception:
                scan_data = None
            if scan_data:
                node.publish_scan(sim_time, scan_data)

        # Kamera folgt
        sim.set_camera_view(
            eye=(float(pos[0]) - 2.5, float(pos[1]) - 2.5, 1.6),
            target=(float(pos[0]), float(pos[1]), 0.3),
        )

        if float(pos[2]) < FALL_Z:
            print("[WARN] Sturz erkannt -> Reset.")
            reset_robot()
            last_action.zero_()

        step += 1

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    simulation_app.close()

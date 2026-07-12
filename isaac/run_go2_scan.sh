#!/usr/bin/env bash
# run_go2_tf.sh — Go2-Controller mit /cmd_vel-Eingang + /clock /odom /tf-Ausgabe.
# In-Prozess-rclpy: Bridge-Extension richtet das interne jazzy-rclpy selbst ein,
# ABER librmw_implementation.so braucht seine Geschwister-Libs (z.B. libament_index_cpp.so)
# auf LD_LIBRARY_PATH -> sonst "ROS2 Bridge startup failed" + ModuleNotFoundError: rclpy.
source ~/NVIDIA_Omniverse/isaacsim_env/bin/activate
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

# jazzy/lib der Kopie, die das laufende Isaac tatsaechlich laedt (site-packages/exts/3),
# robust per Glob ermittelt (kein hartkodierter Versionspfad):
BRIDGE_LIB=$(ls -d ~/NVIDIA_Omniverse/isaacsim_env/lib/python3.11/site-packages/isaacsim/kit/data/Kit/Isaac-Sim/*/exts/*/isaacsim.ros2.bridge-*/jazzy/lib 2>/dev/null | head -1)
if [ -z "$BRIDGE_LIB" ]; then
  echo "[run_go2_tf] WARNUNG: Bridge jazzy/lib nicht gefunden — Pfad pruefen!"
else
  echo "[run_go2_tf] BRIDGE_LIB = $BRIDGE_LIB"
fi
export LD_LIBRARY_PATH="${BRIDGE_LIB}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd ~/IsaacLab
./isaaclab.sh -p "$SCRIPT_DIR/play_go2_ros_scan.py" "$@"

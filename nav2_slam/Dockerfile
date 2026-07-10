FROM ros:jazzy-ros-base

# SLAM Toolbox + TF-Tools + RViz2 (additiv, sonst minimal)
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-slam-toolbox \
      ros-jazzy-tf2-tools \
      ros-jazzy-rviz2 \
 && rm -rf /var/lib/apt/lists/*

# Host<->Container-DDS ueber UDPv4 (SHM aus) -- Fast-DDS-Versions-Skew Isaac-Bundle <-> Jazzy-Debian
COPY fastdds_udp.xml /root/fastdds_udp.xml
COPY mapper_params_go2.yaml /root/mapper_params_go2.yaml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/fastdds_udp.xml \
    FASTDDS_DEFAULT_PROFILES_FILE=/root/fastdds_udp.xml \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_DOMAIN_ID=0

# ros2 in `docker exec`-Shells verfuegbar machen (exec umgeht den Entrypoint)
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc

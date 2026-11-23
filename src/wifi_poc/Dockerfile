FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    iw \
    wireless-tools \
    python3-pip \
    python3-numpy \
    python3-scipy \
    net-tools \
    build-essential \
    python3-colcon-common-extensions \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-rosidl-default-runtime \
    ros-jazzy-std-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws/src
COPY wifi_msgs/ /ros2_ws/src/wifi_msgs/

WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

WORKDIR /app
COPY wifi_scanner.py /app/wifi_scanner.py
COPY wifi_heatmap_node.py /app/wifi_heatmap_node.py
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["python3", "-u", "/app/wifi_scanner.py"]
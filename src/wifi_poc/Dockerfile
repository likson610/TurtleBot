FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    iw \
    wireless-tools \
    python3-pip \
    net-tools \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-rosidl-default-runtime \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY wifi_msgs /ros2_ws/src/wifi_msgs

RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

COPY wifi_scanner.py /app/wifi_scanner.py

RUN echo '#!/bin/bash\nsource /opt/ros/jazzy/setup.bash\nsource /ros2_ws/install/setup.bash\nexec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["python3", "-u", "/app/wifi_scanner.py"]
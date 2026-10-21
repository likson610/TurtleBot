FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    iw \
    wireless-tools \
    python3-pip \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY wifi_scanner.py /app/wifi_scanner.py

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["python3", "-u", "/app/wifi_scanner.py"]
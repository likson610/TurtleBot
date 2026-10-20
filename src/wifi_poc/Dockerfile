FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    iw \
    wireless-tools \
    python3-pip \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY wifi_scanner.py /app/wifi_scanner.py

CMD ["python3", "/app/wifi_scanner.py"]
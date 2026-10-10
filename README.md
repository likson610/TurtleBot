# TurtleBot

## 🌟 Kluczowe Funkcjonalności

* **🕹️ Teleoperacja:** Zdalne sterowanie platformą za pomocą kontrolera.
* **🗺️ Mapowanie (SLAM 2D):** Tworzenie map zamkniętego otoczenia w czasie rzeczywistym przy użyciu skanera LIDAR.
* **📍 Fuzja Danych i Odometria:** Precyzyjna lokalizacja dzięki połączeniu danych z enkoderów na kołach oraz modułu IMU z wykorzystaniem filtru Kalmana.
* **📡 Analiza Sygnału WiFi (Heatmap):** Zbieranie i wizualizacja danych o sile sygnału sieci (RSSI) nanoszonych dynamicznie na mapę przestrzenną.

## 🛠️ Stos Technologiczny

* **Oprogramowanie i Frameworki:** ROS 2, C++, Python, Arduino (C/C++)
* **Główne pakiety ROS 2:** `ros2_control`, `Nav2`, `slam_toolbox`, `robot_localization`, `teleop_twist_joy`

## ⚙️ Główne Komponenty Sprzętowe

* **Obliczenia i logika:** Raspberry Pi 5 (8GB) + Arduino Nano
* **Sensoryka:** RPLidar A1M8-R6, IMU MPU6050, Enkodery magnetyczne (zintegrowane z silnikami)
* **Napęd:** 2x Silnik DC 12V (200 RPM) z przekładnią, Sterownik silników L298N
* **Zasilanie:** Akumulator LiPo Gens ace 3S 11,1 V 2200 mAh, Przetwornica step-down LM2596s

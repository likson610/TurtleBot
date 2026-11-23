# TurtleBot3 — instrukcja symulacji

Wymagania: Ubuntu 24.04, ROS 2 Jazzy i Gazebo Harmonic.

## Instalacja

Jeśli symulacja już działa, przejdź do uruchamiania. Na nowym urządzeniu
zainstaluj [ROS 2 Jazzy](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst) i zależności:

```bash
sudo apt update
sudo apt install -y \
  python3-numpy python3-scipy python3-matplotlib \
  ros-jazzy-ros-gz \
  ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-description \
  ros-jazzy-robot-state-publisher ros-jazzy-slam-toolbox \
  ros-jazzy-rviz2 ros-jazzy-nav2-map-server \
  ros-jazzy-tf2-ros-py ros-jazzy-launch-ros ros-jazzy-lifecycle-msgs
```

Otwórz terminal Bash w katalogu zawierającym `start.py` i `view.rviz`.
W repozytorium jest to podkatalog `sim`. Przy przenoszeniu na inne urządzenie
skopiuj cały katalog `sim`.

W każdym nowym terminalu wczytaj środowisko ROS:

```bash
source /opt/ros/jazzy/setup.bash
```

Przy niestandardowej instalacji użyj pliku `setup.bash` z tej instalacji.

## Uruchamianie

Wybierz jeden tryb. Przed zmianą zakończ sterowanie klawiszem **Q**, jeśli
działa, i zatrzymaj symulację przez **Ctrl+C**.

### Jazda ręczna (WASD)

Terminal 1:

```bash
python3 start.py manual
```

Terminal 2:

## Klawiatura
```bash
python3 start.py keys --speed 0.08 --turn 0.3
```

Kliknij drugi terminal. **W/S** to przód i tył, **A/D** to skręt,
**Spacja** zatrzymuje robota, a **Q** kończy sterowanie.

## Kontroler
```bash
python3 controller.py
```

Odpalane w osobnym terminalu, sterowanie lewą gałką: przód, tył i boki (kontroler ps4)

### Jazda do celu

```bash
python3 start.py goals --speed 0.08 --turn 0.3
```

Poczekaj, aż w RViz pojawi się mapa. Wybierz **2D Goal Pose**, kliknij
w rozpoznanym wolnym obszarze i przeciągnij strzałkę.

### Eksploracja

```bash
python3 start.py explore --speed 0.08 --turn 0.3
```

Obserwuj przyrost mapy w RViz. Wskazanie celu przez **2D Goal Pose** przełącza
robota na jazdę do wskazanego punktu. Aby wznowić eksplorację, zatrzymaj sesję
i ponownie uruchom `explore`.

## Wybór świata

Bez dodatkowych opcji używany jest `room.sdf`. Dla większego świata
`room2.sdf` użyj punktu startowego `(-0.6, -2.7)`:

```bash
python3 start.py manual --world ./room2.sdf --x -0.6 --y -2.7
```

W trybie jazdy ręcznej uruchom `keys` w drugim terminalu tak jak wyżej.
W trybach autonomicznych użyj:

```bash
python3 start.py explore --world ./room2.sdf --x -0.6 --y -2.7 \
  --speed 0.08 --turn 0.3
```

Aby jechać do celu wskazanego w RViz, zamień `explore` na `goals`.
Opcja `--no-gui` uruchamia symulację bez okna Gazebo, z dostępem do RViz.
Pozostałe opcje wyświetla `python3 start.py --help`.

## Mapa cieplna WiFi

Opcja `--wifi` uruchamia symulowany pomiar siły sygnału (RSSI) wirtualnego
routera i nanosi pomiary na mapę SLAM. Działa w trybach `manual`, `goals`
i `explore`. Najwygodniej użyć `explore`, bo robot sam pokrywa pomieszczenie:

```bash
python3 start.py explore --wifi --ap 2.5 2.0 --speed 0.08 --turn 0.3
```

`--ap X Y` to pozycja routera we współrzędnych świata Gazebo (tych samych co
`--x --y`), domyślnie `2.5 2.0`. W RViz biała kula oznacza router, a warstwa
**WiFi heatmap** pokazuje średnie RSSI w polach 0.2 × 0.2 m. Kolor czerwony
oznacza silny sygnał (≥ −40 dBm), a niebieski słaby (≤ −90 dBm).

Model sygnału: `RSSI = −40 − 20·log10(d) − Σ tłumienie przeciętych przeszkód + szum (σ = 2 dB)`.
Ściany i przeszkody są odczytywane z pliku świata `.sdf`, więc model sięga
też poza obszar już zmapowany przez SLAM.

Tłumienie zależy od rodzaju przeszkody, rozpoznawanego po wysokości bryły w `.sdf`:

| Przeszkoda | Rozpoznanie | Domyślnie | Flaga |
|---|---|---|---|
| Ściana, filar | wysokość ≥ 0.9 m | 5 dB | `--wall-loss DB` |
| Mebel, skrzynia, regał | wysokość < 0.9 m | 2 dB | `--obstacle-loss DB` |

Każde przecięcie linii router–robot z przeszkodą odejmuje jej tłumienie, więc
za dwiema ścianami sygnał jest słabszy o 2 × `--wall-loss`. Orientacyjne
wartości dla 2.4 GHz: ścianka gipsowa 3–5 dB, cegła 6–10 dB, beton 10–15 dB.
Przykład dla budynku z murowanymi ścianami:

```bash
python3 start.py explore --wifi --world ./room2.sdf --x -0.6 --y -2.7 \
  --wall-loss 10 --obstacle-loss 3 --speed 0.08 --turn 0.3
```

Przy starcie węzeł wypisuje podział, np. `22 przeszkód z room2.sdf: 18 ścian
po 10.0 dB, 4 niższych (meble) po 3.0 dB`. Pozostałe parametry (`tx_power`,
`path_loss_exp`, `noise_std`, `resolution`, `full_height` – próg wysokości
ściany) można zmienić w `wifi_heatmap.py` albo przez `--ros-args -p`
przy samodzielnym uruchomieniu węzła.

Po **Ctrl+C** pomiary trafiają do `wifi_results/wifi_<data>.csv` (x, y, RSSI
w układzie `map`) i `.png` (mapa cieplna na tle mapy SLAM; wymaga
`python3-matplotlib`). Na żywo wartości można podglądać przez
`ros2 topic echo /wifi/rssi`.

### Lokalizacja routera

Flaga `--locate-ap` (włącza też `--wifi`) odwraca zadanie: robot sam szacuje,
gdzie stoi router, na podstawie zebranych pomiarów RSSI.

```bash
python3 start.py explore --locate-ap --ap 2.5 2.0 --speed 0.08 --turn 0.3
```

Co 5 s węzeł dopasowuje model `RSSI = P0 − 10·n·log10(d) − Σ tłumienie ścian`
do średnich RSSI w polach mapy cieplnej. Nieznane są pozycja routera, `P0` i `n`.
Najpierw przeszukuje siatkę kandydatów, potem dopracowuje wynik metodą najmniejszych
kwadratów (`scipy`). W RViz warstwa **WiFi AP estimate** pokazuje fioletową kulę
(oszacowanie), półprzezroczystą elipsę 2σ i błąd względem prawdziwego routera
(białej kuli). W terminalu pojawia się m.in. `WiFi: AP ~ (x, y) map, ±σ m (1 sigma), …, błąd … m`.
Estymacja startuje, gdy jest co najmniej 15 pól z pomiarami rozłożonych na ≥ 1 m.

Po **Ctrl+C** dochodzi plik `wifi_results/wifi_<data>_ap.csv` z historią
oszacowań (liczba pól, pozycja, `P0`, `n`, RMSE, σ, błąd), z którego można
narysować spadek błędu w czasie. Końcowe oszacowanie z elipsą jest też na `.png`.

Lokalizacja przyjmuje to samo tłumienie przeszkód co symulacja (`--wall-loss`,
`--obstacle-loss`). Parametry węzła: `locate_period` (s), `locate_min_cells`,
`locate_wall_loss` (dB, jedna wartość dla wszystkich przeszkód zamiast tłumienia
z symulacji, np. do sprawdzenia skutków błędnego założenia) i `locate_walls` (`false` wyłącza model ścian, co daje wyraźnie gorszy wynik
w pomieszczeniach ze ścianami działowymi).

Ten sam węzeł działa z prawdziwym skanerem z `src/wifi_poc`
(`-p source:=scan -p bssid:=aa:bb:cc:dd:ee:ff`, opcjonalnie `-p locate_ap:=true`;
bez świata `.sdf` model ścian jest pomijany). Wymaga to zbudowanego pakietu `wifi_msgs`.

## Zapis mapy

Zostaw symulację uruchomioną. Otwórz dodatkowy terminal w zapisywalnym
katalogu przeznaczonym na mapy:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ./turtlebot_maps
ros2 run nav2_map_server map_saver_cli -f ./turtlebot_maps/pokoj_01 \
  --ros-args -p use_sim_time:=true
ls ./turtlebot_maps/pokoj_01.*
```

Zapis obejmuje obraz mapy i plik YAML. Kolejny zapis wykonaj pod inną
nazwą, aby zachować ten wynik. Nowa sesja symulacji rozpoczyna mapowanie od zera.

## Gdy symulacja nie działa

| Objaw | Co sprawdzić |
|---|---|
| `ros2: command not found` lub brak `rclpy` | Wczytaj `setup.bash` w tym samym terminalu i sprawdź instalację zależności. |
| Brak pliku świata, `astar` albo `dwa` | Uruchom polecenie w katalogu z pełną kopią plików symulacji. |
| Robot stoi w trybie `manual` | Uruchom `keys` w drugim terminalu, kliknij go i przytrzymaj **W**. |
| Robot nie jedzie do celu | Poczekaj na mapę i wskaż bliski punkt w wolnym obszarze. Sprawdź komunikaty w terminalu. |
| RViz nie pokazuje mapy | Sprawdź, czy symulacja działa i czy w RViz `Fixed Frame` ma wartość `map`. W dodatkowym terminalu możesz sprawdzić publikację mapy przez `ros2 topic info /map --verbose`. |

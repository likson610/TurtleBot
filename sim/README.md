# TurtleBot3 — instrukcja symulacji

Wymagania: Ubuntu 24.04, ROS 2 Jazzy i Gazebo Harmonic.

## Instalacja

Jeśli symulacja już działa, przejdź do uruchamiania. Na nowym urządzeniu
zainstaluj [ROS 2 Jazzy](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst) i zależności:

```bash
sudo apt update
sudo apt install -y \
  python3-numpy python3-scipy \
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

```bash
python3 start.py keys --speed 0.08 --turn 0.3
```

Kliknij drugi terminal. **W/S** to przód i tył, **A/D** to skręt,
**Spacja** zatrzymuje robota, a **Q** kończy sterowanie.

### Jazda do celu

```bash
python3 start_explore.py goal_pose --speed 0.08 --turn 0.3
```

Poczekaj, aż w RViz pojawi się mapa. Wybierz **2D Goal Pose**, kliknij
w rozpoznanym wolnym obszarze i przeciągnij strzałkę.

### Eksploracja

```bash
python3 start_explore.py explore --speed 0.08 --turn 0.3
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
python3 start_explore.py explore --world ./room2.sdf --x -0.6 --y -2.7 \
  --speed 0.08 --turn 0.3
```

Aby jechać do celu wskazanego w RViz, zamień `explore` na `goal_pose`.
Opcja `--no-gui` uruchamia symulację bez okna Gazebo, z dostępem do RViz.
Pozostałe opcje wyświetla `python3 start.py --help`.

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

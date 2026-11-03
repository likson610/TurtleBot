# TurtleBot3 — instalacja i obsługa symulacji


**Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**, w maszynie VMware.

Polecenia wykonuj w **terminalu Ubuntu, w powłoce Bash**. Folder Windows służy
do przenoszenia plików. Program uruchamiaj z kopii na dysku Ubuntu:
**`~/turtlebot-sim`**.

## Spis treści

- [1. Szybki start — gdy instalacja jest gotowa](#1-szybki-start)
- [2. Instalacja ROS 2 i zależności](#2-instalacja-ros-2-i-zależności)
- [3. Przenoszenie i aktualizacja plików na VM](#3-przenoszenie-i-aktualizacja-plików-na-vm)
- [4. Ręczna jazda WASD i mapowanie](#4-wasd)
- [5. Jazda do celu — goals](#5-goals)
- [6. Samodzielna eksploracja — explore](#6-explore)
- [7. Zapis mapy](#7-zapis-mapy)
- [8. Zatrzymywanie i zmiana trybu](#8-zatrzymywanie)
- [9. Opcje i przykłady](#9-opcje)
- [10. Odczytywanie mapy w RViz](#10-rviz)
- [11. Problemy i diagnostyka](#11-problemy)
- [12. Pliki, ustawienia i __pycache__](#12-pliki)

<a id="1-szybki-start"></a>
## 1. Szybki start
Przed zmianą trybu zakończ poprzednią symulację i klawiaturę przez **Ctrl+C**
w ich terminalach. Uruchamiaj jedną symulację naraz.

| Funkcjonalność | Tryb | Terminale |
|---|---|---|
| Sterowanie joystickiem i tworzenie mapy | `manual` + `keys` | 2 |
| Jazda do wskazanego punktu na mapie | `goals` | 1 |
| Samodzielne wybieranie punktów przez robota | `explore` | 1 |
| Zapisać bieżącą mapę | Działająca symulacja + zapis | Dodatkowy terminal |

W każdym nowym terminalu przygotuj środowisko:

```bash
cd ~/turtlebot-sim
source /opt/ros/jazzy/setup.bash
```

Następnie wybierz **jeden** tryb:

**Jazda ręczna** — dodatkowo uruchom `keys` w drugim terminalu, patrz rozdział 4:

```bash
python3 start.py manual
```

**Cel wskazywany w RViz:**

```bash
python3 start.py goals --speed 0.08 --turn 0.3
```

**Automatyczny wybór celów:**

```bash
python3 start.py explore --speed 0.08 --turn 0.3
```

 Do pierwszych prób używaj małej prędkości,
np. `0.08 m/s`; domyślny limit skryptu wynosi `0.15 m/s`.

<a id="2-instalacja-ros-2-i-zależności"></a>
## 2. Instalacja ROS 2 i zależności
Zainstaluj ROS2 Jazzy z: \
[instalacja pakietów ROS 2 Jazzy](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst).

Zależności:
**Wykonaj raz po instalacji ROS.** Możesz powtórzyć polecenie, jeśli brakuje pakietów.

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

| Składnik | Zastosowanie |
|---|---|
| `ros-gz` | Gazebo i połączenie symulatora z ROS |
| Pakiety TurtleBot3 | Gotowy model Burgera i jego opis |
| `slam-toolbox` | Tworzenie mapy ze skanów |
| `rviz2` | Podgląd mapy, robota, skanów i trasy |
| `nav2-map-server` | Zapis mapy do plików |
| NumPy, SciPy | Obliczenia na mapie i przeszkodach |

Gazebo dla Jazzy jest dostarczane przez pakiety ROS. Nie musisz dodawać osobnego
repozytorium Gazebo. [Dokumentacja instalacji ROS/Gazebo](https://gazebosim.org/docs/harmonic/ros_installation/).

Sprawdź podstawowe pakiety — każde polecenie powinno wypisać ścieżkę:

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix turtlebot3_gazebo
ros2 pkg prefix slam_toolbox
ros2 pkg prefix ros_gz_sim
```

Ta wersja działa bezpośrednio przez Pythona: **nie wymaga `colcon build`,
`install/setup.bash` ani `teleop_twist_keyboard`.**

<a id="3-przenoszenie-i-aktualizacja-plików-na-vm"></a>

## 3. Przenoszenie i aktualizacja plików na VM

### 3.1. Udostępnianie z Windows

W VMware otwórz **Virtual Machine Settings → Options → Shared Folders**
i włącz udostępnianie folderu:

| Ustawienie | Wartość używana w instrukcji |
|---|---|
| Ścieżka Windows | `D:\Turtlebot-zsd` |
| Nazwa udziału VMware | `Turtlebot-zsd` |
| Folder symulacji w Windows | `D:\Turtlebot-zsd\turtlebot-simple` |
| Ten sam folder w Ubuntu | `/mnt/hgfs/Turtlebot-zsd/turtlebot-simple` |

Jeśli udział ma inną nazwę, podstaw ją w poleceniach. Wielkość liter w Ubuntu
ma znaczenie. [Foldery współdzielone według VMware](https://knowledge.broadcom.com/external/article?legacyId=60262).

Sprawdź widoczność plików:

```bash
ls /mnt/hgfs/Turtlebot-zsd/turtlebot-simple
```

Powinny być widoczne: `start.py`, `navigation.py`, `astar.py`, `dwa.py`,
`room.sdf`, `view.rviz` i `README.md`.

Jeśli udziału nie ma, sprawdź ustawienia VMware i obecność jego narzędzi:

```bash
sudo apt install -y open-vm-tools open-vm-tools-desktop
```

Jeśli udostępnianie jest włączone, ale `/mnt/hgfs` nie jest zamontowane
(sprawdź przez `mountpoint /mnt/hgfs`), zamontuj je na bieżącą sesję:

```bash
sudo mkdir -p /mnt/hgfs
sudo vmhgfs-fuse .host:/ /mnt/hgfs -o subtype=vmhgfs-fuse,allow_other
```

To polecenie nie utrwala montowania na kolejne uruchomienia VM.
[Instrukcja montowania VMware](https://knowledge.broadcom.com/external/article?legacyId=60262).

### 3.2. Pierwsza kopia na dysku Ubuntu

Wykonaj, jeśli **`~/turtlebot-sim` jeszcze nie istnieje**:

```bash
cp -r /mnt/hgfs/Turtlebot-zsd/turtlebot-simple ~/turtlebot-sim
cd ~/turtlebot-sim
ls
```

Jeśli katalog już istnieje, użyj aktualizacji poniżej. Ponowne `cp -r` do
istniejącego katalogu może utworzyć dodatkowy podfolder.

### 3.3. Aktualizacja istniejącej kopii

Zatrzymaj symulację. Jeśli zmieniałeś pliki na VM, zachowaj ich kopię przed
nadpisaniem. Następnie:

```bash
cp /mnt/hgfs/Turtlebot-zsd/turtlebot-simple/{start.py,navigation.py,astar.py,dwa.py,room.sdf,view.rviz,README.md} ~/turtlebot-sim/
```

Kopiuj `navigation.py`, `astar.py` i `dwa.py` razem. Zmiany w Windows nie
aktualizują automatycznie kopii `~/turtlebot-sim`.

<a id="4-wasd"></a>
## 4. WASD

**Cel:** ręcznie jeździć robotem i obserwować tworzenie mapy.

**Terminal 1 — świat, robot, SLAM i RViz:**

```bash
cd ~/turtlebot-sim
source /opt/ros/jazzy/setup.bash
python3 start.py manual
```

**Terminal 2 — sterowanie klawiaturą:**

```bash
cd ~/turtlebot-sim
source /opt/ros/jazzy/setup.bash
python3 start.py keys --speed 0.08 --turn 0.3
```

Kliknij terminal 2 i przytrzymuj klawisz:

| Klawisz | Działanie |
|---|---|
| **W** | Przód |
| **S** | Tył |
| **A / D** | Obrót w lewo / w prawo |
| **Spacja** | Natychmiastowe wysłanie STOP |
| **Q / Ctrl+C** | STOP i zakończenie klawiatury |

Małe i wielkie litery działają tak samo. Obowiązuje ostatnie polecenie;
łączenie `W+A` nie jest obsługiwane. Po **0.6 s bez kolejnego znaku** program
wysyła STOP. Puszczenie klawisza lub utrata fokusu może więc zatrzymać robota
z takim opóźnieniem. Spacja wysyła STOP od razu.

`manual` uruchamia symulację, a `keys` wysyła kierunek i prędkość. Samo
zwiększenie `--speed` nie wprawia robota w ruch. Przy jeździe ręcznej operator
omija przeszkody; kontrola drogi hamowania dotyczy `goals` i `explore`.

<a id="5-goals"></a>
## 5. Goals

**Cel:** wskazać punkt w RViz i pozwolić robotowi do niego dojechać.
Zatrzymaj poprzednią symulację oraz klawiaturę. Wystarczy jeden terminal:

```bash
cd ~/turtlebot-sim
source /opt/ros/jazzy/setup.bash
python3 start.py goals --speed 0.08 --turn 0.3
```

1. Poczekaj na mapę i skany w RViz. `Fixed Frame` powinno mieć wartość `map`.
2. Wybierz narzędzie **2D Goal Pose** na pasku RViz.
3. Na jasnym, rozpoznanym obszarze kliknij i przeciągnij myszą, a potem puść.
4. Obserwuj trasę i ruch Burgera. Pierwszy cel wybierz blisko, w otwartej przestrzeni.

Obecny algorytm realizuje **pozycję XY**. Końcowej orientacji strzałki nie ustawia.
Narzędzie `2D Pose Estimate` nie służy do wskazywania celu.

Nowy cel zastępuje poprzedni — nie tworzy kolejki punktów. Cel na przeszkodzie
lub poza znanym obszarem może zostać przesunięty do pobliskiego wolnego pola
albo odrzucony. Wybieraj jasne pola z odstępem od ścian.

Robot planuje trasę przez A*, a prędkości dobiera przez DWA. Zmiana mapy może
wywołać STOP i przeliczenie trasy. LIDAR, aktualność odometrii i przestrzeń
potrzebna do hamowania są sprawdzane dodatkowo. Błędna mapa lub lokalizacja
nadal mogą uniemożliwić prawidłowy przejazd.

<a id="6-explore"></a>
## 6. Explore

**Cel:** samodzielny wybór kolejnych punktów odkrywania mapy.
Zatrzymaj poprzednią symulację i klawiaturę, następnie:

```bash
cd ~/turtlebot-sim
source /opt/ros/jazzy/setup.bash
python3 start.py explore --speed 0.08 --turn 0.3
```

Nie wskazujesz celu. Robot szuka granicy poznanego i nieznanego obszaru,
wybiera osiągalny punkt po poznanej stronie, dojeżdża i ponawia wybór.
Komunikat **`Auto-Exploring to reachable frontier at (...)`** oznacza wybór celu.
Obserwuj trasę i przyrost mapy w RViz.

Uwzględniane są wszystkie granice połączone z pozycją robota. Gdy nie ma
osiągalnego celu, robot zatrzymuje się i czeka na nowe dane. Komunikat podaje
**liczbę dostępnych pól i pól granicy**. Nie jest to pewność ukończenia mapy:
przyczyną może być odcięty obszar, margines przy ścianach lub błąd mapowania.
Tryb jest eksperymentalny i może utknąć w trudnym układzie przeszkód.

Wskazanie **2D Goal Pose** podczas eksploracji wyłącza automatyczny wybór celów
i przełącza robota na jazdę do wskazanego punktu. Powrót do eksploracji wymaga
zakończenia symulacji i ponownego uruchomienia `explore`.
**Nie uruchamiaj `keys` równocześnie z `goals` ani `explore`.**

<a id="7-zapis-mapy"></a>
## 7. Zapis mapy

Pozostaw symulację i SLAM uruchomione. Przy jeździe ręcznej najpierw zatrzymaj
robota spacją. W **dodatkowym terminalu**:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/turtlebot_maps
ros2 run nav2_map_server map_saver_cli -f ~/turtlebot_maps/pokoj_01 \
  --ros-args -p use_sim_time:=true
```

Powstaną plik YAML i obraz mapy (zwykle PGM). Aby zachować poprzedni zapis,
użyj kolejnej nazwy, np. `pokoj_02`. Sprawdź wynik:

```bash
ls ~/turtlebot_maps/pokoj_01.*
```

Zapis kończy działanie po utworzeniu plików; symulacja nadal pracuje.
[Opis Map Saver CLI — Nav2 Jazzy](https://api.nav2.org/nav2-jazzy/html/main__cli_8cpp_source.html).

Obraz i YAML nie zapisują całej sesji SLAM. Obecny `start.py` nie ma opcji
wczytania zapisanej mapy do lokalizacji i jazdy. Ponowne uruchomienie symulacji
rozpoczyna nowe mapowanie; zapisane pliki pozostają na dysku.

<a id="8-zatrzymywanie"></a>
## 8. Zatrzymywanie

| Co zatrzymujesz | Co zrobić |
|---|---|
| Ruch przy ręcznej jeździe | Spacja w terminalu `keys` |
| Klawiaturę | Q lub Ctrl+C w terminalu `keys` |
| Jazdę autonomiczną i symulację | Ctrl+C w terminalu `goals` / `explore` |
| Całą sesję ręczną | Zakończ `keys`, potem Ctrl+C w terminalu `manual` |
| Polecenie diagnostyczne | Ctrl+C w jego własnym terminalu |

Poczekaj na zakończenie procesów przed następnym uruchomieniem. Przełączenie
trybu przez restart tworzy nową sesję symulacji i mapowania.

<a id="9-opcje"></a>
## 9. Opcje

Opcje dopisuj do polecenia, np. `python3 start.py goals --speed 0.08`.
Używaj kropki dziesiętnej: **`0.08`**, a nie `0,08`.

| Opcja | Domyślnie | Znaczenie |
|---|---|---|
| `manual` | Domyślny tryb | Symulacja do ręcznej jazdy |
| `keys` | — | WASD dla już działającej symulacji |
| `goals` | — | Cel z RViz |
| `explore` | — | Samodzielne wybieranie celów |
| `--speed` | `0.15` m/s | Limit w `goals`/`explore` lub prędkość WASD; `0 < speed ≤ 0.22` |
| `--turn` | `0.7` rad/s | Limit/prędkość skrętu; `0 < turn ≤ 2.84` |
| `--no-gui` | Wyłączone | Gazebo bez okna; fizyka i sensory pracują |
| `--no-rviz` | Wyłączone | Pomija RViz |
| `--hardware-rendering` | Wyłączone | Pomija wymuszenie renderowania na CPU |
| `--world ŚCIEŻKA` | `room.sdf` obok skryptu | Własny świat SDF |
| `--x`, `--y` | `0.0`, `0.0` m | Pozycja startowa; wybierz miejsce bez przeszkód |
| `--help` | — | Pomoc i zakończenie programu |

`--speed` i `--turn` w poleceniu `manual` nie ustawiają klawiatury w drugim
terminalu — dopisz je do `keys`. W trybie `keys` opcje okien i świata nie mają
zastosowania, bo uruchamiane jest tylko sterowanie.

Po przygotowaniu terminala wybierz potrzebny przykład:

```bash
python3 start.py --help
python3 start.py manual --no-rviz
python3 start.py explore --no-gui --speed 0.08 --turn 0.3
python3 start.py explore --no-gui --no-rviz --speed 0.08 --turn 0.3
python3 start.py manual --x 1.0 --y -1.0
python3 start.py manual --world /home/macdom/moj_swiat.sdf
```

Własny świat musi zapewniać podłoże, fizykę oraz systemy Gazebo potrzebne do
tworzenia modeli i pracy sensorów. Możesz oprzeć go na działającym `room.sdf`.

Domyślnie skrypt ustawia `LIBGL_ALWAYS_SOFTWARE=1` i `QT_QPA_PLATFORM=xcb` —
w tej konfiguracji VMware służyło to ograniczeniu migania. Flaga
`--hardware-rendering` pomija wymuszenie CPU, ale nie usuwa zmiennej ustawionej
wcześniej w terminalu. Do osobnego testu renderowania sprzętowego:

```bash
env -u LIBGL_ALWAYS_SOFTWARE python3 start.py manual --hardware-rendering
```

<a id="10-rviz"></a>
## 10. RViz

| Element | Co pokazuje |
|---|---|
| Gazebo | Wirtualny pokój i faktyczny ruch robota w symulacji |
| RViz: Map | Mapę utworzoną przez SLAM |
| RViz: LaserScan | Punkty wykryte przez LIDAR |
| RViz: Burger (RobotModel) | Model umieszczony według danych ROS/TF |
| RViz: Planned path | Trasę z `/planned_path` |

W dostarczonym widoku jasne pola oznaczają wolny obszar, czarne — przeszkody,
a ciemniejsze szare — obszar nieznany. `Fixed Frame` powinno mieć wartość `map`.
Kolorystykę można zmienić w RViz.

LIDAR ma ograniczony zasięg, a przeszkody zasłaniają obszar za sobą. Brak mapy
w oddali nie musi być błędem: obszar będzie odkrywany w miarę przemieszczania
się robota. Trasa przebiega po poznanym wolnym obszarze z marginesem od ścian.

<a id="11-problemy"></a>
## 11. Problemy

| Objaw | Co sprawdzić |
|---|---|
| `ros2: command not found`, brak `rclpy` lub `launch` | Wykonaj `source /opt/ros/jazzy/setup.bash` w tym samym terminalu; sprawdź instalację |
| Brak `astar`, `dwa` albo pliku świata | Skopiuj wszystkie siedem plików do jednego folderu |
| `manual` działa, robot stoi | Uruchom `keys` w drugim terminalu, kliknij go i przytrzymaj W |
| Robot stoi w `goals` | Poczekaj na mapę, wskaż bliski cel przez `2D Goal Pose`; sprawdź komunikaty STOP/braku trasy |
| `brak osiągalnej granicy mapy` | Explore nie ma dostępnego celu; zachowaj podane liczby do diagnozy |
| `brak wolnego pola startowego` | Robot wypada w zablokowanej części mapy lub jej otoczeniu |
| Stary tekst `Could not find a reachable frontier` | Sprawdź aktualizację plików i restart procesów; nowa wersja ma inną diagnostykę |
| Brak `/map` | Sprawdź, czy symulacja i SLAM nadal działają |
| Miganie lub błędny obraz | Wróć do uruchomienia bez `--hardware-rendering`; sprawdź także, czy mapa jest publikowana |

Jeśli cel został wybrany, ale robot stoi, sprawdź sensory, odometrię i pauzę
Gazebo. Nieaktualne dane lub brak poprawnego TF zatrzymują autonomiczną jazdę;
nie każdy taki warunek wypisuje osobne ostrzeżenie.

Jeśli mapa nakłada się w różnych obrotach, rozpocznij jedną nową sesję i sprawdź
liczbę publikujących `/map`. Powinien być jeden `slam_toolbox`. Jedna publikacja
nie wyklucza błędu lokalizacji — jeśli problem wraca, porównaj skany, odometrię
i TF. Sama zmiana prędkości nie naprawia lokalizacji.

### Diagnostyka podczas pracy symulacji

Otwórz dodatkowy terminal i wykonaj `source /opt/ros/jazzy/setup.bash`.
Potrzebne polecenia uruchamiaj pojedynczo:

```bash
# Działające węzły.
ros2 node list

# Publikujący i odbiorcy mapy.
ros2 topic info /map --verbose

# Częstotliwość skanów — zakończ pomiar przez Ctrl+C.
ros2 topic hz /scan

# Jedna wiadomość odometrii: prędkości.
ros2 topic echo /odom --once --field twist.twist

# Czy płynie czas symulacji.
ros2 topic echo /clock --once

# Kto publikuje polecenia jazdy.
ros2 topic info /cmd_vel --verbose
```

`echo` może czekać na dane — przerwij je przez Ctrl+C, jeśli nic nie nadchodzi.
Do zgłoszenia problemu zachowaj polecenie uruchomienia, fragment logu i wynik
`ros2 node list`. Komunikaty graficzne RViz i nawigacji rozpatruj osobno:
mapa może być publikowana nawet wtedy, gdy jej obraz jest niepoprawny.

<a id="12-pliki"></a>
## 12. Pliki

| Plik | Gdzie szukać ustawień i zmian |
|---|---|
| `start.py` | Uruchamianie, WASD, opcje, most ROS/Gazebo, słownik `SLAM` |
| `navigation.py` | ROS, mapa, sensory, cele, eksploracja i nadzorowanie jazdy |
| `astar.py` | A*, osiągalność pól, współrzędne mapy i sprawdzanie trasy |
| `dwa.py` | `DWAConfig`, dobór prędkości, przewidywanie ruchu i hamowanie |
| `room.sdf` | Pokój 8 × 6 m, podłoże, ściany i przeszkody |
| `view.rviz` | Domyślny widok RViz |
| `README.md` | Ta instrukcja |

W `DWAConfig` są m.in. przyspieszenia, promień robota, margines od przeszkód,
zakres skanów, horyzont przewidywania i wagi oceny ruchu. Maksymalne prędkości
są nadpisywane przez `--speed` i `--turn`. Po zmianach kodu uruchom program ponownie.

**`__pycache__` tworzy automatycznie Python.** Przechowuje przetworzone moduły
`.pyc`, co może przyspieszać kolejne wczytywanie. Nie przyspiesza samej jazdy.
Możesz go usunąć; zostanie odtworzony. Nie trzeba kopiować go na VM.
Aby wyłączyć zapis także w podprocesach uruchamianych przez `start.py`:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 start.py explore --speed 0.08 --turn 0.3
```
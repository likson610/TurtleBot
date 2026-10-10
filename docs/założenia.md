# Pełne założenia projektowe: Turtlebot

## 1. Założenia Mechaniczne i Konstrukcyjne
*   **Wydruk 3D:** Samodzielne wydrukowanie potrzebnych części mechanicznych, w tym elementów podwozia (chassis)
*   **Montaż:** Złożenie całego układu mechanicznego robota 

## 2. Założenia Elektroniczne i Sprzętowe
*   **Montaż elektroniki:** Zaprojektowanie, rozmieszczenia i przylutowanie odpowiednich elementów elektronicznych ze sobą
*   **Sensoryka:** Wyposażenie robota w niezbędne czujniki:
    *   Skaner laserowy (LIDAR) o zasięgu ok. 10m.
    *   Moduł inercyjny (IMU).
    *   Enkodery na wałach silników (do pomiaru odometrii kół).

## 3. Komunikacja i Sterowanie
*   **Teleoperacja:** Możliwość manualnego sterowania platformą
*   **Łączność bezprzewodowa:** Komunikacja robota z kontrolerem oparta na Bluetooth.
*   **Interfejs sterujący:** Wykorzystanie kontrolera do gier (np. pad od Xbox) jako urządzenie do sterowania prędkością pojazdu.

## 4. Oprogramowanie i Fuzja Danych
*   **Niskopoziomowe sterowanie:** Napisanie oprogramowania do obsługi silników.
*   **Fuzja sensoryczna:** Wykorzystanie filtru Kalmana do połączenia danych z różnych źródeł (odometria z kół + dane z IMU).
*   **Cel fuzji:** Zwiększenie dokładności odczytów odometrycznych robota

## 5. Nawigacja, Mapowanie i Lokalizacja
*   **SLAM 2D:** Tworzenie mapy 2D, statycznej i ograniczonej przestrzeni wokół robota w czasie rzeczywistym (real-time).
*   **Ograniczenia sprzętowe mapowania:** Zasięg tworzenia mapy zależy bezpośrednio od parametrów sprzętowych skanera LIDAR i wynosi w przybliżeniu 10 metrów.
*   **Wymagania dokładnościowe:** System musi lokalizować robota z dokładością 90%

## 6. Analiza Propagacji Sygnału (WiFi Heatmap)
* **Monitorowanie RSSI:** Ciągły odczyt wskaźnika siły odbieranego sygnału (RSSI - Received Signal Strength Indicator) z sieci WiFi (między robotem a routerem).
* **Generowanie Mapy Zasięgu:** Stworzenie systemu, który w czasie rzeczywistym przetwarza i wizualizuje te dane w postaci przestrzennej "mapy cieplnej" (heatmap), obrazującej rozkład natężenia sygnału w środowisku roboczym.

## 7. Wymagania i Ograniczenia Środowiska Operacyjnego
* **Przestrzeń zamknięta:** Robot jest przeznaczony do pracy wyłącznie w pomieszczeniach (indoor). Skanowany obszar musi posiadać fizyczne granice (ściany). System nie jest przystosowany do nawigacji w otwartej, nieskończonej przestrzeni.
* **Statyczność otoczenia:** Przestrzeń robocza musi być w pełni statyczna. W trakcie mapowania w obszarze działania robota nie mogą znajdować się poruszające obiekty (ludzie, zwierzęta domowe), które zakłócałyby odczyty LIDAR-u i proces budowania mapy.
* **Nawierzchnia i trakcja:** Podłoże musi być twarde i płaskie. Tarcie między kołami a podłożem musi umożliwiać płynny ruch przy zachowaniu limitu mocy i momentu obrotowego na wałach silników, aby nie doprowadzić do ich przeciążenia. Wyklucza się poruszanie po dywanach i wykładzinach o wysokim/grubym włosiu.
* **Optyka otoczenia:** Z przestrzeni roboczej należy usunąć lustra, szyby oraz inne powierzchnie silnie odbijające światło znajdujące się na linii wzroku sensora. Odbicia mogłyby generować fałszywe odczyty, artefakty na mapie i powodować błędy skanera.
* **Ograniczenia wysokościowe przeszkód (martwa strefa):** Z uwagi na specyfikę skanowania 2D, wszystkie przeszkody w otoczeniu muszą przecinać płaszczyznę skanowania. Wszelkie obiekty, których wysokość jest mniejsza niż punkt zamocowania modułu LIDAR na robocie, nie zostaną wykryte i stanowią obszar zagrożenia kolizją. Zakłada się, że przestrzeń robocza nie posiada obiektów niższych niż punkt zamocowania czujnika LIDAR.

## 8. Symulacja i Testowanie Wirtualne (Gazebo)
- **Model cyfrowy:** Opracowanie platformy w środowisku symulacyjnym, uwzględniającej wymiary, właściwości fizyczne (masa, inercja), kinematykę układu oraz bryły kolizyjne robota.
- **Środowisko wirtualne:** Stworzenie w środowisku Gazebo zamkniętej przestrzeni testowej, odpowiednio odzwierciedlającej zakładane środowisko operacyjne (ściany, statyczne przeszkody) w celu symulacji nawigacji typu indoor.
- **Cel symulacji:** Bezpieczne rozwijanie, testowanie i optymalizacja algorytmów (SLAM, Nav2, fuzja EKF) w środowisku wirtualnym, przed ich docelowym wdrożeniem na rzeczywistym robocie, co znacząco zminimalizuje ryzyko kolizji i uszkodzeń sprzętu.

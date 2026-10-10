# Milestone 1
**Cel główny:** Złożenie platofrmy, polutnowanie wszystkich niezbędnych części elektronicznych oraz implementacja podstawowego sterowania za pomocą kontrolera bezprzewodowego.

## 1. Mechanika i Konstrukcja 
*   **Wydruk 3D:** Wydruk wszystkich niezbędnych elementów konstrukcyjnych robota
*   **Montaż:** Złożenie wydrukowanych elementów mechanicznych wraz z silnikami i kołami w jedną, zwartą i sztywną platformę, gotową do umieszczenia na niej elektroniki.

## 2. Elektronika
*   **Rozmieszczenie komponentów:** Ułożenie elementów elektronicznych (komputer pokładowy, Arduino, sterownik silników, moduły zasilania) wewnątrz obudowy robota.
*   **Lutowanie i okablowanie:** Wykonanie połączeń elektrycznych (lutowanie).

## 3. Oprogramowanie Niskopoziomowe
*   **Kod sterujący silnikami:** Implementacja kodu (firmware) na mikrokontroler (Arduino). Kod będzie odpowiedzialny za bezpośrednią obsługę sterowników silników na podstawie komend otrzymywanych z wyższej warstwy systemu sterowania.
*   **Enkodery:**  Implementacja kodu do zczytywania impulsów z enkoderów kół

## 4. Oprogramowanie Wysokopoziomowe i Sterowanie (ROS 2)
*   **Wdrożenie ros2_control:** Skonfigurowanie warstwy abstrakcji sprzętowej (`ros2_control`), tak aby system ROS 2 mógł płynnie komunikować się z Arduino wysyłając żądania prędkości obrotowej kół.
*   **Konfiguracja łączności bezprzewodowej:** Poprawne sparowanie i utrzymanie stabilnego połączenia Bluetooth między komputerem pokładowym robota a kontrolerem 

## 5. Wstępna Implementacja Analizy Sygnału WiFi (Statyczna Heatmapa)

* **Monitorowanie RSSI:** Implementacja modułu oprogramowania do ciągłego pobierania i przetwarzania wartości siły sygnału (RSSI) z interfejsu sieciowego.
* **Statyczna Wizualizacja (Brak lokalizacji):** Przygotowanie kodu generującego mapę cieplną (heatmapę). Z racji braku aktywnego systemu lokalizacji przestrzennej na tym etapie prac, system będzie rejestrował i wizualizował dane o sygnale w sposób statyczny. Moduł udowodni zdolność systemu do przetwarzania danych RSSI, jednak na tym etapie pomiary nie będą jeszcze korelowane ze zmieniającymi się w czasie współrzędnymi robota na mapie.
  
## 8. Symulacja i Testowanie Wirtualne (Gazebo)
- **Model cyfrowy:** Opracowanie platformy w środowisku symulacyjnym, uwzględniającej wymiary, właściwości fizyczne (masa, inercja), kinematykę układu oraz bryły kolizyjne robota.
- **Środowisko wirtualne:** Stworzenie w środowisku Gazebo zamkniętej przestrzeni testowej, odpowiednio odzwierciedlającej zakładane środowisko operacyjne (ściany, statyczne przeszkody) w celu symulacji nawigacji typu indoor.
- **Dodanie i wstępna implementacja mapowania 2D oraz SLAM**

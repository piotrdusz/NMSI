# Wprowadzenie do ROS 2 i symulacji w Stage

Ta instrukcja jest **częścią wprowadzającą** do laboratorium z planowania trasy algorytmem A\*. Za jej wykonanie nie ma osobnych punktów, ale bez niej nie da się sensownie zrobić części punktowanej (`02_a_star.md`). Zakładamy, że nie miałeś wcześniej styczności z ROS 2.

## 1. Cel i przebieg laboratorium

Celem całego laboratorium jest uzupełnienie programu, który w symulacji planuje trasę mobilnego robota po mapie za pomocą algorytmu A\*. Robot ma:

1. otrzymać cel wskazany myszą w RViz2,
2. wyznaczyć trasę z aktualnej pozycji do celu (**to jest Twoje zadanie**),
3. przejechać po niej dzięki gotowemu węzłowi `path_follower`.

Laboratorium składa się z dwóch części:

| Plik | Zawartość | Punkty |
|------|-----------|--------|
| `01_wprowadzenie_ros2.md` (ten plik) | podstawy ROS 2, praca z symulacją z poziomu terminala | brak |
| `02_a_star.md` | teoria A\*, pseudokod, zadania programistyczne | tak |

W ćwiczeniu korzystamy z następujących narzędzi:

| Narzędzie | Rola |
|-----------|------|
| **ROS 2 (Lyrical)** | komunikacja między programami, narzędzia CLI |
| **Stage** (pakiet `stage_ros2`) | prosty symulator 2D robota z lidarem, publikuje odometrię i skan |
| **RViz2** | wizualizacja danych (mapa, skan, trasa) i wskazywanie celu |
| **Nav2: `map_server`, `amcl`** | udostępnienie mapy i lokalizacja robota na mapie |
| **colcon** | budowanie pakietów |
| **C++17, VS Code** | język i edytor |

Ta instrukcja składa się z dwóch etapów. Najpierw (punkty 2 i 3) poznasz pojęcia ROS 2 i architekturę naszego ćwiczenia, a następnie (punkt 4) sprawdzisz je w praktyce na działającej symulacji. Każde ćwiczenie w punkcie 4 ma ten sam układ: **Zrób** (polecenia do wpisania), **Zaobserwuj** (czego szukać w wyniku) i **Zastanów się** (pytania, które mają pokazać, co się stało i dlaczego). Pod pytaniami znajduje się rozwijane wyjaśnienie — zajrzyj do niego dopiero po własnej próbie odpowiedzi.

## 2. Podstawowe pojęcia ROS 2

ROS 2 (Robot Operating System 2) nie jest systemem operacyjnym. To zestaw bibliotek, narzędzi i konwencji, który pozwala wielu niezależnym programom (np. sterownik silników, lokalizacja, planer trasy) wymieniać dane bez pisania własnej komunikacji. Pełna dokumentacja: <https://docs.ros.org/en/lyrical/ROS-Framework.html>.

### 2.1. Węzły (nodes)

Węzeł to pojedynczy działający program realizujący jedno konkretne zadanie, np. obsługę lidaru albo planowanie trasy. Cały system składa się z wielu takich węzłów, które działają równolegle. Każdy węzeł ma nazwę (np. `/a_star`), po której można go odnaleźć w systemie. W naszym ćwiczeniu węzłami są m.in. symulator, RViz2, `map_server`, `amcl`, `map_processor`, `a_star` i `path_follower`. Twoim zadaniem będzie rozbudowa węzła `a_star`.

### 2.2. Tematy i wiadomości (topics, messages)

Węzły wymieniają dane głównie za pomocą **tematów** w modelu **publikuj–subskrybuj**:

- **temat** (topic) to nazwany kanał, np. `/cmd_vel`, przez który przesyłane są wiadomości jednego, ustalonego typu,
- **publisher** wysyła wiadomości na temat,
- **subscriber** odbiera je i każdą nową wiadomość obsługuje w funkcji zwrotnej (*callback*).

Nadawca i odbiorca nie muszą o sobie nic wiedzieć — łączy ich wyłącznie nazwa tematu, a na jednym temacie może być wielu nadawców i odbiorców. Dzięki temu można na przykład podmienić symulator na prawdziwego robota bez zmian w pozostałych programach.

**Wiadomość** (message) to struktura danych o ustalonych polach. Na przykład `geometry_msgs/msg/Twist` zawiera prędkość liniową `linear` i prędkość kątową `angular`. Definicję dowolnego typu wiadomości można wyświetlić poleceniem `ros2 interface show`.

Poza tematami ROS 2 oferuje jeszcze **serwisy** (zapytanie i odpowiedź) oraz **akcje** (długotrwałe zadania z informacją o postępie), ale w tym ćwiczeniu używamy wyłącznie tematów.

### 2.3. Pakiety i przestrzeń robocza

**Pakiet** (package) to podstawowa jednostka organizacji kodu: zawiera kod, opis zależności (`package.xml`), konfigurację budowania (`CMakeLists.txt`), pliki launch i konfiguracyjne. Pakiety leżą w katalogu `src` **przestrzeni roboczej** (workspace):

```
~/nmsi_ws/
├── src/
│   └── NMSI/
│       ├── nmsi_lab/      <- pakiet z ćwiczeniem
│       ├── stage_ros2/    <- integracja symulatora Stage z ROS 2
│       └── ...
├── build/    <- tworzone przez colcon
├── install/  <- tworzone przez colcon
└── log/      <- tworzone przez colcon
```

Kod edytujesz tylko w `src`. Katalogi `build`, `install` i `log` tworzy narzędzie `colcon` podczas budowania pakietów i nie należy ich modyfikować ręcznie. Przestrzeń robocza jest ładowana automatycznie w każdym nowym terminalu, dlatego po zbudowaniu pakietu ROS od razu znajduje zbudowane programy.

### 2.4. Parametry i pliki launch

Zachowanie węzła można zmieniać bez ingerencji w jego kod za pomocą **parametrów**. Przykładem jest `robot_radius` węzła `map_processor`, który określa, o ile mają zostać „pogrubione" przeszkody na mapie. Parametry przekazuje się przy uruchamianiu węzła, najczęściej z pliku YAML (u nas `config/nav_config.yaml`) albo bezpośrednio w pliku launch.

Uruchamianie każdego węzła osobno byłoby uciążliwe, dlatego używa się **plików launch** (`launch/*.launch.py`), które uruchamiają wiele węzłów naraz i ustawiają ich parametry. W ćwiczeniu są dwa takie pliki:

- `simulation.launch.py` uruchamia symulator, RViz2, serwer mapy i lokalizację; nie modyfikujesz go,
- `robot_controller.launch.py` uruchamia węzły `a_star` i `path_follower`; to dzięki niemu Twój kod trafi do działającego systemu.

### 2.5. TF — układy współrzędnych

Ta sama pozycja ma inne współrzędne zależnie od tego, względem czego ją opisujemy: inne względem mapy, a inne względem samego robota. Dlatego robot i jego otoczenie opisuje się w wielu **układach współrzędnych**, a zależności między nimi publikuje mechanizm **TF** (na temacie `/tf`). W ćwiczeniu pojawiają się trzy układy:

| Układ | Znaczenie |
|-------|-----------|
| `map` | globalny, nieruchomy układ związany z mapą; w nim planujemy trasę |
| `odom` | układ, w którym pozycję robota wyznacza się z odometrii (ruchu kół); z czasem narasta w niej błąd, czyli „dryf" |
| `base_link` | układ przymocowany do robota, porusza się razem z nim |

Układy tworzą łańcuch: `map` jest układem nadrzędnym względem `odom`, a `odom` względem `base_link`. Każde ogniwo tego łańcucha publikuje inny węzeł. Przekształcenie `odom → base_link`, czyli pozycję robota policzoną z odometrii, publikuje symulator. Przekształcenie `map → odom` publikuje węzeł `amcl`, który porównuje odczyty lasera z mapą i na tej podstawie koryguje dryf odometrii. TF potrafi złożyć oba przekształcenia i podać pozycję robota bezpośrednio w układzie `map`. Właśnie tak szkielet `a_star.cpp` wyznacza punkt startowy (`lookupTransform("map", "base_link", ...)`), nie wiedząc nic o odometrii ani lokalizacji.

### 2.6. Czas symulacji

Symulacja nie musi biec dokładnie w tempie rzeczywistego czasu, dlatego węzły w tym ćwiczeniu mają ustawiony parametr `use_sim_time: true`. Oznacza on, że zamiast zegara komputera używają czasu publikowanego przez symulator na temacie `/clock`. W praktyce `now()` w kodzie węzła zwraca czas symulacji, o czym warto pamiętać, gdy w logach zobaczysz nietypowe znaczniki czasu.

### 2.7. Mapa jako `nav_msgs/msg/OccupancyGrid`

Mapa w ROS 2 to siatka komórek o stałym rozmiarze, przesyłana jako wiadomość `nav_msgs/msg/OccupancyGrid`. Każda komórka mówi, czy jest wolna, zajęta, czy nieznana. Dokładną budowę wiadomości i sposób numerowania komórek omawia `02_a_star.md`. Tu wystarczy wiedzieć, że w ćwiczeniu będą dostępne trzy wersje mapy; dwie ostatnie węzeł `map_processor` tworzy na podstawie pierwszej:

| Temat | Kto publikuje | Zawartość |
|-------|---------------|-----------|
| `/map` | `map_server` | oryginalna mapa z pliku (wolne / zajęte / nieznane) |
| `/map_dilated` | `map_processor` | mapa z przeszkodami „pogrubionymi" o promień robota — robota można traktować jak punkt |
| `/map_cost` | `map_processor` | koszt 0–100 rosnący w miarę zbliżania się do przeszkód |

## 3. Architektura ćwiczenia

Zanim zaczniemy eksperymentować, warto zobaczyć, jak wszystkie węzły są ze sobą połączone. Poniższy diagram pokazuje węzły (prostokąty) i tematy (podpisy strzałek). Strzałka prowadzi od węzła, który publikuje dane, do węzła, który je subskrybuje. Węzeł `a_star` jest wyróżniony, bo to jego uzupełnisz w części punktowanej.

```mermaid
flowchart LR
    stage[stage<br/>symulator]
    rviz[rviz2]
    ms[map_server]
    mp[map_processor]
    amcl[amcl]
    astar[a_star<br/>TWÓJ KOD]
    pf[path_follower]

    ms -- /map --> mp
    ms -- /map --> amcl
    mp -- /map_dilated --> astar
    mp -- /map_cost --> astar
    stage -- /base_scan --> amcl
    stage -- /odom, /tf --> amcl
    amcl -- "/tf (map→odom)" --> astar
    rviz -- /goal_pose --> astar
    astar -- /path --> pf
    astar -- /path --> rviz
    pf -- /cmd_vel --> stage
    rviz -- "/cmd_vel (panel Teleop)" --> stage
```

Diagram można odczytać jako historię jednego zadania dla robota:

1. Wskazujesz cel narzędziem **2D Goal Pose** w RViz2, a RViz2 publikuje go na temat `/goal_pose`.
2. Węzeł `a_star` odbiera cel, pyta TF o aktualną pozycję robota na mapie i wyznacza trasę, którą publikuje na temat `/path` (docelowo, gdy uzupełnisz jego kod). Do planowania wykorzystuje mapy `/map_dilated` i `/map_cost` przygotowane przez `map_processor`.
3. Węzeł `path_follower` odbiera `/path` i co 100 ms publikuje na `/cmd_vel` prędkość (typ `geometry_msgs/msg/Twist`), która prowadzi robota do kolejnego punktu trasy.
4. Symulator wykonuje ruch robota i publikuje nową odometrię oraz odczyty lasera. Na ich podstawie `amcl` na bieżąco aktualizuje położenie robota na mapie, więc przy następnym celu `a_star` znowu dostanie aktualną pozycję.

Na tym samym temacie `/cmd_vel` może publikować również panel Teleop w RViz2, co pozwala sterować robotem ręcznie.

W szkielecie, który dostajesz, `a_star` zamiast trasy publikuje na razie prosty odcinek od startu do celu. Zobaczysz to w punkcie 4.10.

## 4. Praca z symulacją

W tej części sprawdzisz opisane wcześniej pojęcia na działającej symulacji. Przy każdym ćwiczeniu najpierw wykonujesz polecenia (**Zrób**), potem sprawdzasz, co się stało (**Zaobserwuj**), a na końcu zastanawiasz się nad przyczynami (**Zastanów się**).

Polecenia wpisujesz w terminalu Guake, który rozwija się i chowa klawiszem **F12**. Do pracy potrzebujesz kilku terminali jednocześnie, więc każdy nowy otwieraj jako **nową kartę** Guake (**Ctrl+Shift+T**). Procesy działające w danej karcie, np. symulację, można zatrzymać klawiszami **Ctrl+C**.

### 4.1. Uruchomienie symulacji

**Zrób.** W pierwszej karcie terminala wpisz:

```bash
ros2 launch nmsi_lab simulation.launch.py world:=hallway
```

**Zaobserwuj.** Po kilku sekundach pojawią się dwa okna: **Stage** (symulator z widokiem świata i robotem) oraz **RViz2** (z gotową konfiguracją: mapa, odczyty lasera, układy TF, trasa i panel Teleop). Mapa w RViz2 pojawi się z krótkim opóźnieniem. W terminalu zobaczysz komunikaty kilku węzłów. Parametr `world:=` wybiera świat: `cave` (domyślny), `hallway` albo `lines`.

**Zastanów się.** Jednym poleceniem uruchomiłeś kilka programów. Skąd ROS 2 wiedział, które uruchomić? Gdzie jest to zapisane?

<details>
<summary>Wyjaśnienie</summary>

Polecenie `ros2 launch <pakiet> <plik>` wykonuje plik launch. W `simulation.launch.py` (katalog `launch` pakietu `nmsi_lab`) wymienione są węzły `stage`, `rviz2`, `map_server`, `map_processor`, `amcl` i `lifecycle_manager`. Ostatni z nich uruchamia `map_server` i `amcl` dopiero po starcie pozostałych, stąd opóźnienie mapy.

</details>

Tej karty nie zamykaj, bo działa w niej cała symulacja. Symulację można zatrzymać klawiszami Ctrl+C w tej karcie; nie zamykaj okien Stage i RViz2 krzyżykiem.

### 4.2. Węzły

**Zrób.** Otwórz nową kartę i wpisz:

```bash
ros2 node list
ros2 node info /map_processor
```

**Zaobserwuj.** Pierwsze polecenie wypisuje nazwy działających węzłów; znajdź na liście `/stage`, `/rviz2`, `/map_server` i `/amcl`. Drugie wypisuje tematy, które wskazany węzeł subskrybuje (sekcja *Subscribers*) i publikuje (sekcja *Publishers*).

**Zastanów się.** Z jakich tematów korzysta `map_processor` i co z nich robi?

<details>
<summary>Wyjaśnienie</summary>

`map_processor` subskrybuje `/map` i na jej podstawie publikuje dwie nowe mapy: `/map_dilated` i `/map_cost`. Sam nie komunikuje się z robotem — jest pośrednikiem, który przygotowuje dane dla planera.

</details>

### 4.3. Tematy

**Zrób.** W tej samej karcie wpisz kolejno:

```bash
ros2 topic list -t
ros2 topic info /base_scan --verbose
ros2 topic hz /base_scan
```

Polecenie `hz` działa bez końca — przerwij je klawiszami Ctrl+C.

**Zaobserwuj.** Pierwsze polecenie wypisuje wszystkie tematy razem z typami wiadomości; znajdź wśród nich `/map`, `/map_dilated`, `/base_scan`, `/cmd_vel`, `/odom` i `/tf`. Drugie pokazuje typ tematu `/base_scan` oraz nazwy węzłów, które go publikują i subskrybują. Trzecie podaje, ile razy na sekundę pojawia się nowa wiadomość (*average rate*).

**Zastanów się.** Który węzeł publikuje `/base_scan`, a które go odbierają? Do czego mogą im służyć te dane?

<details>
<summary>Wyjaśnienie</summary>

Odczyty lasera publikuje symulator (`/stage`). Odbiera je `amcl`, który porównuje je z mapą, aby określić położenie robota, oraz RViz2, który rysuje je na ekranie jako punkty. Dane z lasera publikowane są cyklicznie, niezależnie od tego, czy robot się porusza.

</details>

### 4.4. Wiadomości i mapa

**Zrób.** Wyświetl definicje dwóch typów wiadomości, a potem metadane mapy:

```bash
ros2 interface show geometry_msgs/msg/Twist
ros2 interface show nav_msgs/msg/OccupancyGrid
ros2 topic echo /map --once --field info
```

Pełnej mapy nie wypisuj — jej tablica `data` ma dziesiątki tysięcy elementów, dlatego wyświetlamy tylko pole `info`. Jeśli `echo` nic nie wypisuje, poczekaj chwilę na załadowanie mapy albo dodaj opcje `--qos-durability transient_local --qos-reliability reliable` (mapa jest publikowana tylko raz, więc odbiorca musi o nią w ten sposób poprosić).

**Zaobserwuj.** Definicja `Twist` składa się z dwóch wektorów: `linear` i `angular`. W definicji `OccupancyGrid` znajdź pola `info` i `data`. Wynik trzeciego polecenia zawiera `resolution`, `width`, `height` i `origin`. Zapisz sobie te wartości — będą potrzebne w zadaniu.

**Zastanów się.** Ile metrów ma mapa wzdłuż osi x? Co oznacza `origin`?

<details>
<summary>Wyjaśnienie</summary>

Szerokość mapy w metrach to `width` razy `resolution` (rozmiar jednej komórki w metrach) i powinna odpowiadać rozmiarowi świata (dla `hallway` 25 m). Pole `origin` określa położenie w układzie `map` lewego dolnego rogu mapy, czyli punktu, od którego liczone są komórki.

</details>

### 4.5. Graf węzłów i tematów

**Zrób.** W nowej karcie uruchom narzędzie rysujące połączenia między węzłami:

```bash
rqt_graph
```

**Zaobserwuj.** Pojawi się wykres, na którym węzły są połączone strzałkami opisanymi nazwami tematów. Jeśli widzisz tylko węzły, wybierz w górnym menu widok *Nodes/Topics (all)* i odśwież wykres przyciskiem w lewym górnym rogu.

**Zastanów się.** Porównaj wykres z diagramem z punktu 3. Których węzłów jeszcze brakuje i dlaczego?

<details>
<summary>Wyjaśnienie</summary>

Brakuje `a_star` i `path_follower`, ponieważ uruchamia je dopiero drugi plik launch (`robot_controller.launch.py`). Do tego czasu temat `/path` może mieć odbiorcę (RViz2), który czeka na dane, ale nie ma jeszcze nadawcy.

</details>

### 4.6. Ręczne sterowanie robotem

**Zrób.** Robot stoi w miejscu, bo nikt nie wysłał mu polecenia jazdy. W RViz2 zaznacz w panelu Teleop pole *Enabled* i poruszaj robotem za pomocą pola sterowania. Jednocześnie w nowej karcie terminala wpisz:

```bash
ros2 topic echo /odom --field pose.pose.position
```

Następnie odznacz *Enabled* i sprawdź, kto pracuje na temacie sterowania:

```bash
ros2 topic info /cmd_vel --verbose
```

**Zaobserwuj.** Robot rusza w oknie Stage i w RViz2, a współrzędne `x` i `y` wypisywane w terminalu zmieniają się wraz z jego ruchem. Przerwij `echo` klawiszami Ctrl+C. Drugie polecenie pokazuje, które węzły publikują i subskrybują `/cmd_vel`.

**Zastanów się.** Skąd symulator wiedział, że ma jechać, i skąd wzięły się nowe współrzędne w terminalu?

<details>
<summary>Wyjaśnienie</summary>

Panel Teleop publikuje na `/cmd_vel` wiadomości typu `Twist`, a symulator je subskrybuje i nadaje robotowi taką prędkość. Symulator publikuje też odometrię na `/odom`, czyli pozycję robota policzoną z jego ruchu — to ją wypisywało polecenie `echo`.

</details>

**Zrób.** To samo polecenie można wysłać ręcznie z terminala. Przy wyłączonym panelu Teleop wpisz:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

Po kilku sekundach przerwij je klawiszami Ctrl+C. Potem uruchom je ponownie z `angular: {z: 0.5}` i znowu przerwij.

**Zaobserwuj.** Pierwsze polecenie powinno ruszyć robota prosto, drugie po łuku. Po przerwaniu publikowania robot nie zatrzymuje się od razu — po kilku sekundach w terminalu z symulacją pojawia się komunikat `watchdog timeout`.

**Zastanów się.** Dlaczego robot jechał jeszcze po przerwaniu polecenia? Co z tego wynika dla programu, który ma zatrzymać robota?

<details>
<summary>Wyjaśnienie</summary>

Symulator pamięta ostatnie otrzymane polecenie i wykonuje je, aż dostanie nowe. Dopiero po 5 sekundach bez żadnych poleceń (parametr `base_watchdog_timeout`) zatrzymuje robota. Dlatego program sterujący, taki jak `path_follower`, na końcu trasy publikuje jawnie zerową prędkość.

</details>

> **Uwaga.** Na `/cmd_vel` powinno publikować tylko jedno źródło naraz. Przed uruchomieniem `path_follower` wyłącz panel Teleop i przerwij wszystkie polecenia `ros2 topic pub`, w przeciwnym razie robot dostanie sprzeczne polecenia.

### 4.7. Układy współrzędnych (TF)

**Zrób.** Uruchom narzędzie wypisujące pozycję robota w układzie mapy, a w tym czasie pojeździj robotem za pomocą panelu Teleop:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

Przerwij je klawiszami Ctrl+C, wyłącz Teleop i wpisz:

```bash
ros2 topic info /tf --verbose
ros2 run tf2_tools view_frames
```

**Zaobserwuj.** `tf2_echo` co sekundę wypisuje położenie (*Translation*) i orientację (*Rotation*) układu `base_link` względem `map`; wartości zmieniają się, gdy robot jedzie. Drugie polecenie pokazuje węzły publikujące na `/tf`. Trzecie zapisuje w bieżącym katalogu plik `frames_*.pdf` z drzewem układów — otwórz go.

**Zastanów się.** Które węzły publikują na `/tf` i które ogniwo drzewa układów dostarcza każdy z nich? Dlaczego można zapytać o pozycję `base_link` w układzie `map`, mimo że żaden węzeł nie publikuje bezpośrednio takiego przekształcenia?

<details>
<summary>Wyjaśnienie</summary>

Symulator (`/stage`) publikuje `odom → base_link` oraz położenie czujników na robocie, a `amcl` publikuje `map → odom`. TF składa przekształcenia w łańcuch `map → odom → base_link`, więc pozycję robota w układzie mapy można odczytać tak, jakby została opublikowana bezpośrednio. W ten sam sposób robi to `a_star`.

</details>

### 4.8. Parametry

**Zrób.** Wpisz:

```bash
ros2 param list /map_processor
ros2 param get /map_processor robot_radius
```

**Zaobserwuj.** Pierwsze polecenie wypisuje parametry węzła (m.in. `robot_radius`, `max_cost_distance` i `occupied_threshold`), a drugie wartość wybranego parametru.

**Zastanów się.** Gdzie ta wartość została ustawiona i co oznacza w odniesieniu do robota?

<details>
<summary>Wyjaśnienie</summary>

Wartość ustawiono w `simulation.launch.py` w definicji węzła `map_processor`. Jest to promień robota w metrach: węzeł pogrubia przeszkody o tę odległość, dzięki czemu robota można później traktować jak punkt. Węzeł odczytuje parametr jeden raz przy starcie, więc zmiana wartości w trakcie działania nie przebudowałaby map.

</details>

### 4.9. Mapy w RViz2

**Zrób.** W panelu *Displays* po lewej stronie RViz2 rozwiń wyświetlacz **Map**. W polu **Topic** zmieniaj kolejno temat na `/map`, `/map_dilated` i `/map_cost`. Możesz też włączać i wyłączać wyświetlacz polem wyboru, aby widzieć mapę i sam świat.

**Zaobserwuj.** Czym różnią się trzy mapy w pobliżu ścian i przeszkód? Jeśli po zmianie tematu mapa się nie wyświetla, rozwiń pole *Topic* i sprawdź, czy *Durability Policy* ma wartość *Transient Local*.

**Zastanów się.** Dlaczego planer ma korzystać z `/map_dilated`, a nie z oryginalnej `/map`? Do czego może służyć `/map_cost`?

<details>
<summary>Wyjaśnienie</summary>

Na `/map_dilated` przeszkody są powiększone o promień robota. Jeśli środek robota znajduje się w wolnej komórce tej mapy, robot nie dotyka ściany, więc planując trasę można go traktować jak punkt. Mapa `/map_cost` zawiera koszt od 0 (daleko od przeszkody) do 100 (przy przeszkodzie); pozwala planerowi preferować trasy biegnące z dala od ścian.

</details>

### 4.10. Budowanie i uruchomienie szkieletu programu

**Zrób.** Zbuduj pakiet z ćwiczeniem:

```bash
cd ~/nmsi_ws
colcon build --symlink-install --packages-select nmsi_lab
```

**Zaobserwuj.** Jeśli budowanie się powiodło, na końcu pojawi się podsumowanie `Summary: 1 package finished`. W razie błędu szukaj pierwszej linii zawierającej `error:`. Po **każdej** zmianie kodu C++ trzeba ponownie zbudować pakiet i zrestartować węzły (Ctrl+C w karcie z `robot_controller.launch.py` i ponowne uruchomienie).

**Zrób.** Przy działającej symulacji i wyłączonym panelu Teleop uruchom w nowej karcie węzły ze szkieletu:

```bash
ros2 launch nmsi_lab robot_controller.launch.py
```

W RViz2 wybierz narzędzie **2D Goal Pose** na górnym pasku, kliknij punkt na mapie po drugiej stronie przeszkody i przeciągnij kursor (kierunek nie ma tu znaczenia). Potem w innej karcie wpisz:

```bash
ros2 topic echo /path --once
```

**Zaobserwuj.** W RViz2 pojawi się zielona trasa `/path`, a w terminalu `path_follower` komunikaty (`Path registered`, a po dojechaniu `Goal achieved...`). Robot jedzie prosto do celu, a jeśli po drodze jest ściana, zatrzymuje się na niej. Wynik `echo` pokazuje listę `poses` zawierającą tylko dwa punkty. Jeśli robot utknie, zatrzymaj go klawiszami Ctrl+C w karcie z `robot_controller.launch.py`.

**Zastanów się.** Dlaczego robot nie omija przeszkód? Co musi zrobić `a_star`, żeby robot dojechał do celu bezpiecznie?

<details>
<summary>Wyjaśnienie</summary>

Szkielet nie planuje żadnej trasy: publikuje tylko punkt startowy i cel. `path_follower` nie zna mapy i po prostu jedzie do kolejnych punktów trasy, więc omijanie ścian musi być zapewnione przez samą trasę. Zadaniem `a_star` jest więc zastąpienie tej prostej linii serią punktów, które prowadzą przez wolne komórki mapy.

</details>

To właśnie będzie Twoje zadanie w części punktowanej: wyznaczyć trasę algorytmem A\*.

## 5. Pytania kontrolne

Przed przejściem do `02_a_star.md` sprawdź, czy potrafisz odpowiedzieć na poniższe pytania. Jeśli nie, wróć do odpowiedniego ćwiczenia z punktu 4.

1. Czym różni się węzeł od tematu? Kto publikuje, a kto subskrybuje `/goal_pose`?
2. Jaki typ wiadomości jest używany na `/cmd_vel` i które jego pola sterują robotem?
3. W którym układzie współrzędnych `a_star` planuje trasę i skąd zna pozycję robota w tym układzie?
4. Co robi `map_processor` i czym różni się `/map_dilated` od `/map_cost`?
5. Jak sprawdzić w terminalu rozdzielczość i rozmiar mapy?
6. Dlaczego robot ze szkieletu programu jedzie przez ściany?

## 6. Najczęstsze problemy

| Problem | Rozwiązanie |
|---------|-------------|
| `ros2: command not found` | Otwórz nową kartę terminala. Jeśli problem nie znika, zgłoś to prowadzącemu. |
| `Package 'nmsi_lab' not found` | Zbuduj pakiet (punkt 4.10) i otwórz nową kartę terminala. |
| Zmiana w kodzie nie działa | Zbuduj pakiet ponownie i uruchom węzły od nowa. |
| W RViz2 nie ma mapy | Poczekaj kilka sekund, bo mapa pojawia się z opóźnieniem. Sprawdź, czy w wyświetlaczu Map wybrany jest właściwy temat i czy *Durability Policy* ma wartość *Transient Local*. |
| `Could not transform base_link to map` | Zwykle `amcl` jeszcze się nie uruchomił. Poczekaj chwilę lub sprawdź `ros2 run tf2_ros tf2_echo map base_link`. |
| Robot jedzie chaotycznie | Na `/cmd_vel` publikuje więcej niż jedno źródło (Teleop, `ros2 topic pub`, `path_follower`). Zostaw tylko jedno. |
| Po zamknięciu okna Stage lub RViz2 coś nadal działa | Procesy kończ klawiszami Ctrl+C w karcie, w której zostały uruchomione. Zamknięcie samego okna nie zatrzymuje pozostałych węzłów. |

## 7. Ściąga poleceń

| Polecenie | Opis |
|-----------|------|
| `ros2 launch nmsi_lab simulation.launch.py world:=hallway` | symulacja (`cave`, `hallway`, `lines`) |
| `ros2 launch nmsi_lab robot_controller.launch.py` | węzły `a_star` i `path_follower` |
| `ros2 node list` / `ros2 node info <węzeł>` | węzły |
| `ros2 topic list -t` | tematy z typami |
| `ros2 topic info <temat> --verbose` | typ oraz węzły publikujące i subskrybujące |
| `ros2 topic echo <temat> --once` | jedna wiadomość |
| `ros2 topic hz <temat>` | częstotliwość |
| `ros2 topic pub --rate 10 <temat> <typ> "<dane YAML>"` | publikowanie z terminala |
| `ros2 interface show <typ>` | definicja wiadomości |
| `ros2 param list/get <węzeł> [parametr]` | parametry |
| `ros2 run tf2_ros tf2_echo <z> <do>` | przekształcenie TF |
| `ros2 run tf2_tools view_frames` | drzewo układów w pliku PDF |
| `rqt_graph` | graf węzłów i tematów |
| `colcon build --symlink-install --packages-select nmsi_lab` | budowanie pakietu |

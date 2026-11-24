# NMSI — Q-learning: lokalna realizacja trasy

W poprzednim ćwiczeniu robot dostawał trasę wyznaczoną algorytmem A\* i jechał po niej dzięki gotowemu, „ręcznie" napisanym regulatorowi. Tym razem **regulator zastąpisz agentem uczonym metodą Q-learning**: robot sam nauczy się, jak podążać za trasą, nie znając modelu swojej dynamiki ani żadnego wzoru na sterowanie.

## 1. Cel ćwiczenia

Węzeł `path_follower` odbiera trasę z tematu `/path` (`nav_msgs/msg/Path`) i ma ją „lokalnie realizować": w każdym kroku korzysta z punktu trasy wybranego metodą opisaną w sekcji 4.1 i wybiera jedną z kilku dyskretnych akcji (zestaw prędkości), aby dojechać do końca trasy bez kolizji i bez zjechania z niej.

Szkielet węzła `path_follower` (subskrypcje, publikacje, serwisy, reset robota, losowanie celów, odbiór trasy, wyszukiwanie punktu na trasie i timer) jest gotowy w C++ i Pythonie. Do wyznaczania trasy użyj rozwiązania A* z poprzedniego zadania — możesz podmienić nim kod planisty w C++ albo Pythonie i uruchomić odpowiadający mu launch. Jeśli Twoje rozwiązanie A* nie działa, poproś prowadzącego o kod planisty. Planista odbiera cel z `/goal_pose` (`geometry_msgs/msg/PoseStamped`) i publikuje trasę na `/path` (`nav_msgs/msg/Path`). Lidar nie jest używany przez `path_follower`; kolizję odczytuje on z `/collision`. Do napisania w wybranym pliku `nmsi_lab/src/path_follower.cpp` albo `nmsi_lab/src/path_follower.py` zostały funkcje oznaczone `TODO`:

| # | Funkcja | Zadanie |
|---|---------|---------|
| 1 | `discretize_state` | zamiana ciągłej obserwacji na dyskretny stan |
| 2 | `choose_action` | wybór akcji (strategia ε-zachłanna) |
| 3 | `execute_action` | zamiana akcji na prędkości i publikacja na `/cmd_vel` |
| 4 | `update_q_table` | aktualizacja Q-tablicy |
| 5 | `save_q_table` | zapis Q-tablicy do pliku |
| 6 | `load_q_table` | wczytanie Q-tablicy z pliku |
| 7 | `learning_loop` | jeden krok uczenia (wywoływany z timera) |

Uruchomienie (dwa terminale):

```bash
# terminal 1
ros2 launch nmsi_lab simulation.launch.py world:=cave
# terminal 2: uruchom path_follower oraz planistę A*
ros2 launch nmsi_lab robot_controller.launch.py
```

Wybierz launch według języka planisty i węzła realizującego trasę:

| Launch | A* | path_follower |
|--------|----|---------------|
| `robot_controller.launch.py` | C++ | C++ |
| `robot_controller_python.launch.py` | Python | C++ |
| `robot_controller_follower_python.launch.py` | C++ | Python |
| `robot_controller_all_python.launch.py` | Python | Python |

Na przykład: `ros2 launch nmsi_lab robot_controller_all_python.launch.py`. Każdy launch uruchamia po jednym planiscie i jednym `path_follower`; nie uruchamiaj jednocześnie kilku wariantów. Przykładowy planista i funkcje TODO w obu wersjach `path_follower` wymagają własnej implementacji. Jeżeli masz rozwiązanie A* z poprzedniego zadania, użyj go zamiast przykładowego planisty. Jeśli Twój planista nie działa, poproś prowadzącego o kod planisty.

Budowanie po zmianach: `colcon build --packages-select nmsi_lab && source install/setup.bash` (w katalogu głównym przestrzeni roboczej).

Do nauki zacznij od świata `cave`. Gdy rozwiązanie będzie działać, sprawdź je również w `hallway` i `lines`.

## 2. Punktacja i zasady

### 2.1. Punktacja

| Zadanie | Opis | Punkty |
|---------|------|--------|
| 1 | Dyskretyzacja stanu (`discretize_state`) wraz z uzasadnieniem wyboru przedziałów | 2 |
| 2 | Wybór akcji ε-zachłanny i wykonanie akcji (`choose_action`, `execute_action`) | 1,5 |
| 3 | Aktualizacja Q-tablicy (`update_q_table`) | 1,5 |
| 4 | Zapis i wczytanie Q-tablicy (`save_q_table`, `load_q_table`) | 1 |
| 5 | Krok uczenia: nagrody, warunki końca epizodu, przechowywanie poprzedniego stanu (`learning_loop`) | 2 |
| 6 | Działająca polityka: po nauce robot w trybie „following" dojeżdża do celu bez kolizji w co najmniej 8 z 10 prób | 2 |
| Bonus| Sprawdzić, czy dla różnych wartości parametru ε (który nie zmienia się w czasie uczenia) polityka (Q-tablica) zbiega mniej więcej do tych samych wartości | + 1 |
| Bonus| Sprawdzić, czy prędkość zbiegania do ostatecznej polityki zależu od parametru ε | + 1 |

Kod źródłowy należy przesłać przez platformę **LeON** — wyłącznie pliki źródłowe (bez `build`, `install`, `log`) oraz **plik z nauczoną Q-tablicą**.

- Termin zwykły: **3 stycznia**.
- Termin dodatkowy: **10 stycznia**. Za pracę oddaną w tym terminie przyznaje się 75% uzyskanych punktów.
- Po tych terminach zero punktów.

### 2.2. Samodzielność pracy

> **Zadanie trzeba wykonać samodzielnie.**
>
> Wolno korzystać z: własnej wiedzy, materiałów z wykładu (si8) i przygotowanych przez prowadzącego (w tym tej instrukcji i pseudokodu), dokumentacji ROS 2 oraz biblioteki standardowej wybranego języka (C++ lub Python).
>
> Nie wolno korzystać z: gotowych bibliotek i implementacji uczenia ze wzmocnieniem (np. Stable-Baselines, Gymnasium, PyTorch/TensorFlow), gotowych fragmentów kodu z internetu, kodu innych osób ani **kodu wygenerowanego przez modele językowe (LLM) i inne narzędzia AI**.
>
> Wykrycie pracy wykonanej niesamodzielnie lub z użyciem kodu wygenerowanego przez LLM skutkuje wyzerowaniem punktów za zadanie, utratą punktów bonusowych oraz odjęciem jednego punktu od pozostałych zadań. Prowadzący może poprosić o omówienie napisanego kodu.

W razie wątpliwości, czy coś jest dozwolone, **zapytaj prowadzącego zanim to wykorzystasz**.

## 3. Q-learning — teoria

(Szczegóły: wykład si8.)

### 3.1. Uczenie ze wzmocnieniem

Agent (robot) w kolejnych krokach $t$:

1. obserwuje stan $s_t$,
2. wybiera akcję $a_t$,
3. wykonuje ją, po czym otrzymuje nagrodę $r_t$ i obserwuje nowy stan $s_{t+1}$,
4. uczy się na podstawie krotki $(s_t, a_t, r_t, s_{t+1})$.

Formalnie jest to proces decyzyjny Markowa: zbiór stanów $S$, akcji $A$, funkcja nagrody i funkcja przejścia. Szukamy polityki $\pi: S \to A$ maksymalizującej zdyskontowaną sumę nagród

$$ \sum_{t=0}^{\infty} \gamma^t r_t, \qquad \gamma \in (0, 1]. $$

Parametr $\gamma$ określa, jak bardzo liczą się nagrody odległe w czasie. Zadanie ma charakter **epizodyczny**: epizod kończy się po osiągnięciu celu lub wystąpieniu porażki.

### 3.2. Funkcja Q

$Q(s, a)$ to oczekiwana, zdyskontowana suma nagród, jeśli w stanie $s$ wykonamy akcję $a$, a potem będziemy postępować optymalnie. Znając $Q$, polityka jest prosta: $\pi(s) = \arg\max_a Q(s, a)$. Dla małej liczby stanów i akcji $Q$ przechowujemy w tablicy (Q-tablicy).

### 3.3. Aktualizacja

Po każdym kroku poprawiamy jeden wpis tablicy:

$$ Q(s_t, a_t) \leftarrow Q(s_t, a_t) + \alpha \Big[ r_t + \gamma \max_{a'} Q(s_{t+1}, a') - Q(s_t, a_t) \Big] $$

- $\alpha \in (0, 1]$ — współczynnik uczenia,
- wyrażenie w nawiasie to błąd różnicy czasowej (rzeczywiste „r + γ·max Q" minus dotychczasowe oszacowanie),
- w **stanie terminalnym** nie ma następnego kroku, więc składnik $\gamma \max_{a'} Q(s_{t+1}, a')$ pomijamy (jest równy 0).

Q-learning jest algorytmem **off-policy** i **bezmodelowym**: aktualizacja używa $\max$ po akcjach, niezależnie od tego, jaką akcję agent faktycznie wybierze w następnym kroku, i nie wymaga znajomości funkcji przejścia (w odróżnieniu od programowania dynamicznego).

### 3.4. Eksploracja a eksploatacja

Gdy zawsze wybieramy najlepszą znaną akcję, agent może utknąć w złym rozwiązaniu. Strategia **ε-zachłanna**: z prawdopodobieństwem $\varepsilon$ wybierz akcję losową (eksploracja), z prawdopodobieństwem $1-\varepsilon$ najlepszą wg $Q$ (eksploatacja). Za mały $\varepsilon$ — polityka suboptymalna, za duży — wolna zbieżność. Często $\varepsilon$ zmniejsza się w trakcie uczenia. (Alternatywa: eksploracja Boltzmanna — losowanie akcji z prawdopodobieństwem zależnym od $Q$.)

### 3.5. Parametry i wybory projektowe

- `epsilon` jest już parametrem ROS w szkielecie (domyślnie `0.2`, wartość powinna należeć do zakresu `0`–`1`). Możesz ustawić go przy uruchamianiu, np. w pliku launch/YAML lub przez argumenty ROS; szkielet nie zmniejsza go automatycznie.
- `alpha`, `gamma` oraz wartości/zasady nagród nie są obecnie parametrami ROS ani polami gotowymi w szkielecie. Dobierz je i uzasadnij. Możesz przechowywać je jako pola albo jawnie zadeklarować jako parametry ROS; jeśli wybierzesz parametry ROS, opisz ich nazwy i sposób ustawienia.
- Wartości `alpha` i `gamma` muszą być zgodne z definicjami z wykładu; instrukcja nie narzuca ich konkretnych wartości. Tak samo dobór nagród pozostaje decyzją studenta.

## 4. Dyskretyzacja stanu, akcji i czasu

Q-tablica wymaga **skończonych, małych** zbiorów stanów i akcji. Robot działa w świecie ciągłym, więc trzeba zdecydować trzy rzeczy.

### 4.1. Stan

Funkcja `find_head_point()` wybiera punkt trasy względem robota w układzie `base_link` (szczegóły wyboru poniżej). Wymaganym minimum obserwacji jest kąt do tego punktu (`atan2(y, x)`), aby stan odróżniał kierunek względem trasy. Możesz dodać inne cechy, np. odległość do końca trasy; są one opcjonalne, a nie wymagane przez szkielet. Wybierz i uzasadnij cechy oraz ich przedziały.

`discretize_state` przyjmuje `std::vector<double>`; jego długość i kolejność cech ustala Twoja reprezentacja obserwacji. Minimalną wymaganą cechą jest kąt do punktu zwróconego przez `find_head_point()` (`atan2(y, x)` w `base_link`); odległość do celu jest przykładową cechą dodatkową.

`find_head_point()` zwraca `std::optional<PoseStamped>`: pusty wynik oznacza brak ścieżki albo brak przecięcia ścieżki z okręgiem wyszukiwania. Przeliczenie punktów ścieżki do `base_link` może zgłosić wyjątek TF; funkcja nie zamienia tego wyjątku na pusty wynik. Nie dyskretyzuj brakującej obserwacji ani wartości niefinitywnych i nie podejmuj na ich podstawie akcji. Obsłuż brak punktu i wyjątek TF w miejscu wywołania, zatrzymując ruch, jeśli nie można wiarygodnie wyznaczyć obserwacji. Nie aktualizuj przejścia bez wymaganych danych. Szkielet nie narzuca dalszego rozstrzygnięcia stanu epizodu po błędzie TF.

`find_head_point()` szuka przecięcia z okręgiem o środku w robocie i promieniu `head_radius` (0,4 m). Jeśli koniec trasy jest już w tym okręgu, zwraca koniec. W przeciwnym razie przegląda odcinki trasy od końca w stronę początku i zwraca przecięcie na pierwszym odcinku, który przecina okrąg; gdy odcinek ma dwa przecięcia, preferowane jest to bliższe końcowi trasy. Gdy nie ma takiego punktu, wynik jest pusty. `head_radius` nie określa tolerancji osiągnięcia celu.

Każdą wielkość można podzielić na kilka przedziałów (np. „mocno w lewo / lekko w lewo / prosto / lekko w prawo / mocno w prawo"). Jednym z możliwych sposobów reprezentacji stanu jest wektor indeksów przedziałów (`std::vector<int>`), który w obecnym szkielecie jest kluczem Q-tablicy. To tylko przykład — możesz wybrać inną reprezentację, jeśli lepiej pasuje do Twojego pomysłu. W takim przypadku dostosuj typ `DiscreteState` oraz typ klucza Q-tablicy.

Dobierz reprezentację i przedziały tak, aby liczba stanów była skończona i na tyle mała, by uczenie było wykonalne. Liczba stanów jest iloczynem liczby przedziałów dla poszczególnych wielkości; im więcej stanów, tym dłużej trwa nauka. Przy doborze granic weź pod uwagę fizyczne znaczenie obserwacji (np. `head_radius` = 0,4 m).

### 4.2. Akcje

Zbiór akcji to wybrane przez Ciebie pary prędkości (liniowa, kątowa), np. „prosto", „łagodny skręt w lewo/prawo", „obrót w miejscu w lewo/prawo". `execute_action` zamienia numer akcji na `geometry_msgs/msg/Twist` i publikuje go na `/cmd_vel`. W C++ `action_count` jest stałą kompilacji, a tablica Q używa `std::array<double, action_count>`; w Pythonie `action_count` jest polem klasy, a wartościami tablicy Q są listy. Liczbę akcji dobiera się w kodzie, nie parametrem ROS; liczba i indeksy akcji, tablica Q oraz zapis/odczyt pliku muszą pozostać zgodne. Przed użyciem indeksu do dostępu do Q-tablicy lub mapowania go na prędkość sprawdź, czy akcja należy do zakresu `[0, action_count)`. Nie zakładaj domyślnej akcji dla niepoprawnego indeksu. Wartości prędkości dobierz samodzielnie i eksperymentuj z nimi w symulacji.

### 4.3. Czas

**Nie twórz pętli `for`/`while`, która wielokrotnie wykonuje kroki uczenia, czeka na ruch albo zastępuje timer.** Timer ROS 2 co 100 ms wywołuje `follow_path()`, a ta w trybie jazdy po trasie wywołuje `learning_loop()`. Jedno wywołanie `learning_loop()` obsługuje najwyżej jeden krok/przejście i jedną decyzję. Zwykłe lokalne iterowanie po akcjach, elementach obserwacji lub wierszach pliku jest dozwolone.

Okres 100 ms jest już ustawiony w szkielecie. Zbyt krótki okres może oznaczać, że stan prawie się nie zmienia między decyzjami; zbyt długi może nie dać robotowi dość szybko zareagować na trasę.

## 5. ROS 2 — elementy potrzebne w tym ćwiczeniu

Wcześniej korzystaliśmy z węzłów, tematów i TF. Tu pojawiają się dodatkowo:

### 5.1. Serwisy

Serwis to komunikacja typu pytanie–odpowiedź (żądanie → odpowiedź), w przeciwieństwie do tematów, które są strumieniem danych. Typ `std_srvs/srv/Trigger` ma puste żądanie i odpowiedź z polami `success` i `message`.

```bash
ros2 service list                          # lista serwisów
ros2 service type /start_learning          # typ serwisu
ros2 service call /start_learning std_srvs/srv/Trigger "{}"
```

Zanim uruchomisz uczenie, możesz sprawdzić działanie serwisu bezpośrednio z CLI. Przy uruchomionej symulacji (terminal 1) wywołaj:

```bash
ros2 service call /random_reset std_srvs/srv/Trigger "{}"
```

Serwis przeniesie robota w inne, losowe wolne miejsce świata. To dobry sposób, żeby zobaczyć, jak robot i symulacja reagują na wywołanie serwisu, zanim uruchomisz własny kod.

W szkielecie są serwisy (zaimplementowane, nie zmieniaj ich):

| Serwis | Działanie |
|--------|-----------|
| `/start_learning` | tryb **uczenia**: robot jest resetowany w losowe miejsce, losowany jest cel, węzeł planujący publikuje trasę, uruchamiany jest epizod; po jego końcu — następny |
| `/start_following` | tryb **jazdy po trasie**: zapis Q-tablicy do pliku, robot czeka na cel z `/goal_pose` i jedzie, zawsze wybierając najlepszą akcję |
| `/stop_path_follower` | zatrzymuje robota i czyści trasę |
| `/random_reset` | (Stage) przenosi robota w losowe wolne miejsce |

Program **startuje w trybie jazdy po trasie** (stan `WAITING_FOR_PATH`, `training == false`): nic się nie dzieje, dopóki nie zawołasz `/start_learning` albo nie podasz celu w RViz2.

### 5.2. Tematy dodane w tym ćwiczeniu

```bash
ros2 topic echo /collision      # std_msgs/msg/Bool: true po kolizji robota, false po resecie
ros2 topic echo /path           # trasa z węzła planującego
ros2 topic echo /cmd_vel --once # ostatnie polecenie prędkości
```

W kodzie: `collided` (`std::optional<bool>` w C++, `bool | None` w Pythonie) przechowuje ostatnią wiadomość z `/collision`, `publisher` publikuje na `/cmd_vel`.

#### Krótka lista diagnostyczna

Jeśli robot nie jedzie lub nie pojawia się trasa, sprawdź najpierw, czy uruchomione są oczekiwane węzły i czy tematy publikują dane:

```bash
ros2 topic list
ros2 topic echo /goal_pose --once
ros2 topic info /path
ros2 topic echo /path --once
ros2 topic echo /collision --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /map_dilated --once
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_link
ros2 service list
ros2 param get /path_follower epsilon
```

Wiadomości trasy i mapy używają ramki `map`, a punkt trasy jest przeliczany do `base_link`; sprawdź ramki wiadomości i dostępność transformacji `map` → `base_link`. Sprawdź również, czy `/goal_pose` otrzymuje cel i czy `/path` publikuje trasę. Polecenia pokazują stan systemu, nie zakładają konkretnej częstotliwości ani wartości danych.

### 5.3. Klienci serwisów i pliki

Szkielet pokazuje, jak węzeł sam woła serwisy (`reset_client->async_send_request(...)`). Plik Q-tablicy zapisujesz zwykłym `std::ofstream` — nie jest to nic ROS-owego.

## 6. Wskazówki implementacyjne

### 6.1. Przepływ jednego epizodu

W trybie uczenia szkielet wykonuje (poza `learning_loop`) wszystko sam:

1. reset robota w losowe miejsce → pauza `reset_settle_time`,
2. losowanie celu i publikacja na `/goal_pose` → uruchomiony węzeł planujący publikuje `/path`,
3. pauza `path_settle_time` → od tego momentu timer wywołuje `learning_loop()`,
4. `learning_loop()` kończy epizod przez `finish_episode("powód")`; ta funkcja czyści trasę i zleca kolejny reset (w trybie uczenia dodatkowo zapisuje Q-tablicę).

Epizod powinien się kończyć, gdy:

1. robot ma kolizję (`*collided == true`),
2. robot odjechał zbyt daleko od trasy (`find_head_point()` nie znajduje punktu — okrąg o promieniu `head_radius` nie przecina trasy),
3. robot dojechał do celu w przyjętej tolerancji, mierzonej odległością od robota do końca trasy. `head_radius` (0,4 m) jest promieniem wyszukiwania punktu trasy, **nie** tolerancją celu. Tolerancję celu wybiera student.

Szkielet nie określa kolejności sprawdzania kolizji, zjazdu z trasy i osiągnięcia celu. Zdarzenia mogą wystąpić w tym samym kroku — na przykład robot może jednocześnie znaleźć się w tolerancji celu i zgłosić kolizję — więc zdecyduj, które zdarzenie ma wtedy pierwszeństwo. Nie utożsamiaj braku planu z terminalnym przejściem Q: po wysłaniu losowego celu szkielet czeka na trasę, a po 10 sekundach bez niej próbuje inny cel. Po pięciu nieudanych próbach przechodzi do resetowania pozycji. Nie rozpoczął się wtedy krok jazdy po trasie, więc nie przypisuje się nagrody za wykonane działanie.

### 6.2. Poprzedni stan

Efekt akcji poznajesz dopiero **po** jej wykonaniu. Rozróżnij trzy przypadki (opis kolejności pojęć, nie gotowa implementacja):

1. **Pierwszy poprawny krok epizodu:** nie ma jeszcze `previous_state`, więc nie ma przejścia do aktualizacji. Wybierz i wykonaj akcję, a następnie zapamiętaj stan i akcję.
2. **Zwykły krok:** po wykonaniu poprzedniej akcji znasz nową obserwację. Nagroda za poprzednią akcję należy do przejścia `(previous_state, previous_action) → nowy_stan`; zaktualizuj je jako nieterminalne, a dopiero potem wybierz/wykonaj kolejną akcję i zapamiętaj bieżący stan/akcję.
3. **Krok terminalny:** jeśli po poprzedniej akcji wykryto kolizję, zjazd z trasy lub osiągnięcie celu, przypisz nagrodę terminalną do właśnie zakończonego przejścia `(previous_state, previous_action)`. Zaktualizuj je jako terminalne (bez składnika wartości następnego stanu), zakończ epizod i nie wybieraj kolejnej akcji. Jeżeli epizod kończy się przed wykonaniem jakiejkolwiek akcji, nie ma przejścia do aktualizacji.

Kolizja i brak punktu trasy mogą nie dostarczyć poprawnego stanu następnego; nadal muszą być traktowane jako zdarzenia terminalne, a nie jako stan przekazywany do dyskretyzacji. Gdy osiągnięcie celu jest rozpoznane na podstawie poprawnej obserwacji, ten stan również jest terminalny.

### 6.3. Nagrody

Nagroda jest jedyną informacją o tym, co jest „dobre". Wybierz nagrody i opisz ich wartości/zasady; nie ma jednej skali narzuconej w szkielecie. Pomysły do przemyślenia:

- duża kara za kolizję i zjechanie z trasy, nagroda za dotarcie do celu,
- mała kara za każdy krok (zachęta do szybkiego dojazdu),
- nagroda za zmniejszenie odległości do celu lub za mały kąt do punktu na trasie.

Do liczenia nagrody „za postęp" potrzebujesz wartości z poprzedniego kroku — dodaj odpowiednie pola do klasy. Uważaj na skalę: nagrody za postęp nie powinny przewyższać kary za kolizję.

### 6.4. Zapis i wczytanie

Uzgodnij jeden format tekstowy, który zapis i odczyt stosują identycznie. W szkielecie C++ klucz to `std::vector<int>`, a każdy wiersz Q zawiera `action_count` wartości `double` (obecnie pięć); w Pythonie klucz to `tuple[int, ...]`, a wiersz to lista wartości `float`. Parametr ROS `q_file` ma domyślną nazwę `q_table_path_only.txt`; jeżeli podasz ścieżkę względną, plik jest rozwiązywany względem bieżącego katalogu procesu, a nie automatycznie względem pakietu. Szkielet nie ustala katalogu przechowywania. Zgodność plików między wersjami zależy od samodzielnego uzgodnienia formatu i znaczenia stanów oraz akcji.

Przykładowy format, zgodny z tymi typami: wiersz zawiera najpierw liczbę cech stanu `N`, następnie `N` indeksów całkowitych, a następnie dokładnie `action_count` wartości Q. Przykład dla stanu o dwóch cechach i pięciu akcjach:

```text
2 1 3 0.0 0.25 -0.1 0.0 0.4
```

Wiersz oznacza stan `[1, 3]`, a następnie wartości Q kolejno dla akcji o indeksach `0`–`4`. To format ilustracyjny; możesz wybrać inny, ale opisz go i używaj konsekwentnie. Brak pliku lub pusty plik oznacza pustą Q-tablicę (jest to już wskazane w komentarzu TODO w szkielecie). Dla wiersza niekompletnego, z błędną liczbą pól, niepoprawnymi liczbami lub niefinitywną wartością odrzuć cały wiersz i zgłoś ostrzeżenie; nie wstawiaj częściowo odczytanego stanu. Niepoprawny plik nie może przerwać działania węzła ani spowodować wykonania akcji poza zakresem. `load_q_table` jest wywoływane w konstruktorze; zapis wywołują `finish_episode` oraz `/start_following`.

### 6.5. Zalecana kolejność pracy

1. Wypisz w logu obserwacje i zaimplementuj `discretize_state`; sprawdź ręcznie (`teleop` w RViz2), czy stany zmieniają się sensownie.
2. Zaimplementuj `execute_action` i wywołuj go z `learning_loop` na sztywno — sprawdź, czy robot wykonuje akcje.
3. Zaimplementuj `choose_action` (zachłanny, a potem ε-zachłanny) i `update_q_table`.
4. Napisz kompletną `learning_loop` (kolizja, zjazd z trasy, cel, nagrody).
5. Dodaj zapis i wczytanie; sprawdź, czy po restarcie węzła Q-tablica jest taka sama.
6. Uruchom `/start_learning`; loguj m.in. numer epizodu, wartość ε, wynik epizodu i rozmiar tablicy (`q_table.size()`).
7. Strój $\alpha$, $\gamma$, $\varepsilon$ i nagrody; dopiero potem przełącz na `/start_following` i oceń politykę.

Typowe błędy: aktualizacja Q przed poznaniem następnego stanu, brak obsługi stanu terminalnego, brak resetu `previous_state` po zakończeniu epizodu, ε stałe i zbyt duże/małe, zbyt wiele stanów.

## 7. Sprawdzanie rozwiązania

W rubryce podano wynik 8 udanych dojazdów bez kolizji na 10 prób w trybie `following`. Wynik może zależeć od wybranych parametrów uczenia, nagród, tolerancji celu i akcji. Zapisz te wybory, by umożliwić interpretację wyniku. Poniższe pozycje są sugestiami do samodzielnego sprawdzenia.

| Scenariusz | Oczekiwany rezultat |
|------------|---------------------|
| uczenie w `cave` | obserwuj liczbę udanych epizodów i kolizji |
| `/start_following` + cel z RViz2, prosta trasa | robot dojeżdża do celu |
| `/start_following`, trasa z zakrętami | robot dojeżdża bez kolizji i bez zjechania z trasy |
| restart węzła po nauce | polityka działa bez ponownego uczenia (wczytana Q-tablica) |
| kilka celów po sobie | każdy cel realizowany |
| uruchomienie w `hallway` i `lines` po sprawdzeniu w `cave` | sprawdzenie, jak polityka działa w innych światach |

## 8. Przydatne odnośniki

- Dokumentacja ROS 2 — serwisy: <https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html>
- Q-learning (Wikipedia): <https://en.wikipedia.org/wiki/Q-learning>
- Wykład si8 (materiały z zajęć)

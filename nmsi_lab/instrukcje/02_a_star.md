# Planowanie trasy algorytmem A\*

Przed rozpoczęciem tej części wykonaj `01_wprowadzenie_ros2.md` — zakładamy, że umiesz uruchomić symulację, zbudować pakiet i obserwować tematy w terminalu.

## 1. Cel ćwiczenia

W pliku `nmsi_lab/src/a_star.cpp` znajduje się węzeł `a_star`. Odbiera on cel z RViz2 (`/goal_pose`), wyznacza pozycję startową robota z TF i wywołuje metodę:

```cpp
void AStarCalculator::a_star(PoseStamped start, PoseStamped end);
```

W szkielecie metoda publikuje tylko prosty odcinek od startu do celu. **Twoim zadaniem jest uzupełnić ją tak, aby wyznaczała optymalną trasę na mapie za pomocą algorytmu A\*** i publikowała ją na temat `/path` (typ `nav_msgs/msg/Path`). Resztą (jazda po trasie) zajmuje się gotowy węzeł `path_follower`.

Wszystkie zmiany wykonujesz w `a_star.cpp`. Nie zmieniaj nazw tematów ani pozostałych węzłów.

Uruchamianie (po zbudowaniu pakietu):

```bash
# terminal 1
ros2 launch nmsi_lab simulation.launch.py world:=hallway
# terminal 2
ros2 launch nmsi_lab robot_controller.launch.py
```

## 2. Punktacja i zasady

### 2.1. Punktacja

> Punktacja do uzupełnienia przez prowadzącego.

| Zadanie | Opis | Punkty |
|---------|------|--------|
| 1 | Graf z mapy: konwersje współrzędnych, sprawdzanie dostępności komórek, sąsiedzi | `[do uzupełnienia]` |
| 2 | A\* podstawowy | `[do uzupełnienia]` |
| 3 | A\* zmodyfikowany (ze zbiorem przejrzanych) | `[do uzupełnienia]` |
| Bonus | Uwzględnienie mapy kosztów `/map_cost` | `[do uzupełnienia]` |
| — | `[miejsce na dodatkowe pozycje]` | `[do uzupełnienia]` |

Termin oddania: `[do uzupełnienia]`. Kod (plik `a_star.cpp`) należy przesłać przez platformę zajęć. Wysyłaj wyłącznie pliki źródłowe, bez katalogów `build`, `install` i `log`.

### 2.2. Samodzielność pracy

> **Zadanie trzeba wykonać samodzielnie.**
>
> Wolno korzystać z: własnej wiedzy, materiałów przygotowanych przez prowadzącego (w tym tej instrukcji i pseudokodu), dokumentacji ROS 2 i C++, a także z kontenerów biblioteki standardowej (`std::map`, `std::unordered_map`, `std::priority_queue` itp.).
>
> Nie wolno korzystać z: bibliotek do obsługi grafów i planowania trasy (np. Nav2 planner, LEMON, Boost.Graph), gotowych fragmentów kodu z internetu, kodu innych osób ani **kodu wygenerowanego przez modele językowe (LLM) i inne narzędzia AI**.
>
> Wykrycie pracy wykonanej niesamodzielnie lub z użyciem kodu wygenerowanego przez LLM skutkuje:
>
> - **wyzerowaniem punktów za dane zadanie,**
> - **utratą wszystkich punktów bonusowych** (zarówno z wcześniejszych, jak i z późniejszych zadań),
> - **odjęciem jednego punktu od każdego z pozostałych zadań.**
>
> Prowadzący może poprosić o omówienie napisanego kodu.

W razie wątpliwości, czy coś jest dozwolone, **zapytaj prowadzącego zanim to wykorzystasz**.

## 3. Algorytm A\* — teoria

### 3.1. Grafy

**Graf** to zbiór wierzchołków $V$ oraz zbiór krawędzi $E$ łączących pary wierzchołków. Krawędzie mogą mieć **wagi** (np. odległość lub koszt przejścia). W grafie nieskierowanym krawędź łączy wierzchołki w obie strony. Graf można zapisać m.in. jako macierz sąsiedztwa, listy sąsiedztwa lub macierz incydencji.

W tym ćwiczeniu graf **nie jest dany w całości z góry** — budujemy go na bieżąco na podstawie mapy:

| Element grafu | W naszym zadaniu |
|---------------|------------------|
| wierzchołek | wolna komórka mapy $(x, y)$ |
| krawędź | połączenie komórek sąsiednich (8 kierunków) |
| waga krawędzi | odległość między środkami komórek: $r$ dla ruchu w poziomie/pionie, $r\sqrt{2}$ po skosie ($r$ — rozdzielczość mapy w metrach) |

Sąsiadów komórki wyznaczasz „w locie" w momencie, gdy jej potrzebujesz. Z tego powodu nie ma sensu z góry wypełniać tablicy $g$ nieskończonościami dla wszystkich komórek — mapa `cave` ma ok. 250 tys. komórek. W pseudokodzie poniżej wartości $g$ zapisujemy tylko dla komórek, które faktycznie odwiedziliśmy.

### 3.2. Idea algorytmu

A\* szuka najkrótszej ścieżki z wierzchołka startowego do celu. Dla każdego wierzchołka $x$ utrzymuje:

- $g(x)$ — koszt najlepszej dotychczas znalezionej drogi ze startu do $x$,
- $h(x)$ — **heurystyka**, czyli oszacowanie kosztu drogi z $x$ do celu,
- $f(x) = g(x) + h(x)$ — szacowany koszt całej drogi przechodzącej przez $x$.

W każdym kroku algorytm rozwija wierzchołek o **najmniejszej wartości $f$** spośród dotychczas napotkanych, ale jeszcze nierozwiniętych. Rozwinięcie polega na przejrzeniu sąsiadów i — jeśli przez ten wierzchołek da się do któregoś z nich dojść taniej niż dotychczas — zaktualizowaniu $g$ sąsiada oraz zapamiętaniu, skąd do niego przyszliśmy. Gdy rozwijanym wierzchołkiem jest cel, trasę odtwarzamy cofając się po zapamiętanych poprzednikach.

Dla $h \equiv 0$ A\* sprowadza się do algorytmu Dijkstry. Dobra heurystyka pozwala przeszukać znacznie mniej wierzchołków.

### 3.3. Heurystyka

W tym ćwiczeniu użyj **odległości euklidesowej** od komórki do celu (w metrach, zgodnie z jednostką wag krawędzi):

$$h(x) = \sqrt{(x_1 - x_g)^2 + (y_1 - y_g)^2}$$

gdzie $(x_1, y_1)$ to środek komórki $x$, a $(x_g, y_g)$ to środek komórki docelowej.

Własności heurystyki:

- **dopuszczalna (admissible)** — nigdy nie zawyża rzeczywistego kosztu dojścia do celu: $h(x) \le h^*(x)$. Gwarantuje, że podstawowy A\* znajdzie trasę optymalną.
- **spójna (consistent)** — dla każdej krawędzi $(x, y)$ zachodzi $h(x) \le h(y) + d(x, y)$, gdzie $d$ to waga krawędzi. Gwarantuje, że zmodyfikowany A\* (ze zbiorem przejrzanych) znajdzie trasę optymalną, a każdy wierzchołek wystarczy rozwinąć tylko raz.

Odległość euklidesowa spełnia oba warunki, o ile wagi krawędzi nie są mniejsze niż odległość euklidesowa między końcami krawędzi. Pamiętaj o tym, jeśli zdecydujesz się modyfikować wagi (zadanie bonusowe).

### 3.4. Kolejka priorytetowa

Struktura „zbiór rozpatrywanych" z algorytmu A\* to w praktyce **kolejka priorytetowa**, z której pobieramy element o najmniejszym $f$. Standardowa kolejka (`std::priority_queue`) nie umożliwia zmiany priorytetu elementu już w niej znajdującego się. Typowe rozwiązanie: gdy znajdziemy lepszą drogę do wierzchołka, **wstawiamy go do kolejki ponownie** z nowym priorytetem, a stare wpisy traktujemy jako nieaktualne. W zależności od wersji algorytmu zostaną one pominięte lub przetworzone ponownie (patrz niżej).

### 3.5. Dwie wersje algorytmu

**Wersja podstawowa** nie pamięta, które wierzchołki zostały już rozwinięte. Wierzchołek może więc być rozwijany wielokrotnie, jeśli w międzyczasie znajdziemy do niego lepszą drogę. Trasa jest optymalna, jeśli heurystyka jest dopuszczalna.

**Wersja zmodyfikowana** utrzymuje dodatkowo **zbiór przejrzanych** wierzchołków (rozwiniętych). Rozwinięte wierzchołki nigdy nie są rozpatrywane ponownie. Jest szybsza, ale gwarantuje optymalność tylko dla heurystyki **spójnej**.

## 4. Pseudokod

Oznaczenia: `Q` — kolejka priorytetowa zwracająca element o najmniejszym priorytecie (parę `(wierzchołek, f)`); `g` i `przyszedłZ` — mapy (słowniki), w których **brak klucza oznacza wartość nieskończoną / brak poprzednika** (nie inicjalizujemy ich dla całego grafu); `waga(x, y)` — waga krawędzi; `sąsiedzi(x)` — dostępni sąsiedzi komórki `x`.

### 4.1. Wersja podstawowa

```
funkcja A*(start, meta):
    g := pusta mapa
    przyszedłZ := pusta mapa
    g[start] := 0
    Q := pusta kolejka priorytetowa
    Q.wstaw(start, g[start] + h(start))

    dopóki Q jest niepusta:
        x := Q.zdejmij_minimum()
        jeśli x = meta:
            zwróć zrekonstruuj_trasę(przyszedłZ, meta)
        dla każdego y spośród sąsiedzi(x):
            tymczasowe_g := g[x] + waga(x, y)
            jeśli y nie ma w g lub tymczasowe_g < g[y]:
                przyszedłZ[y] := x
                g[y] := tymczasowe_g
                Q.wstaw(y, g[y] + h(y))
    zwróć porażka
```

Uwagi:

- Warunek „`y` nie ma w `g`" zastępuje porównanie z nieskończonością.
- Ten sam wierzchołek może znajdować się w `Q` wielokrotnie (z różnymi priorytetami). Po zdjęciu z kolejki przetwarzamy go z aktualną (najlepszą znaną) wartością `g[x]`, więc wynik pozostaje poprawny.

### 4.2. Wersja zmodyfikowana (ze zbiorem przejrzanych)

```
funkcja A*(start, meta):
    przejrzane := pusty zbiór
    g := pusta mapa
    przyszedłZ := pusta mapa
    g[start] := 0
    Q := pusta kolejka priorytetowa
    Q.wstaw(start, g[start] + h(start))

    dopóki Q jest niepusta:
        x := Q.zdejmij_minimum()
        jeśli x w przejrzane:
            continue                      // nieaktualny duplikat w kolejce
        jeśli x = meta:
            zwróć zrekonstruuj_trasę(przyszedłZ, meta)
        dodaj x do przejrzane
        dla każdego y spośród sąsiedzi(x):
            jeśli y w przejrzane:
                continue
            tymczasowe_g := g[x] + waga(x, y)
            jeśli y nie ma w g lub tymczasowe_g < g[y]:
                przyszedłZ[y] := x
                g[y] := tymczasowe_g
                Q.wstaw(y, g[y] + h(y))
    zwróć porażka
```

### 4.3. Odtwarzanie trasy

```
funkcja zrekonstruuj_trasę(przyszedłZ, obecny):
    trasa := [obecny]
    dopóki obecny ma wpis w przyszedłZ:
        obecny := przyszedłZ[obecny]
        trasa.dodaj_na_początek(obecny)
    zwróć trasa
```

(Alternatywnie dodawaj na koniec i odwróć listę na końcu.)

### 4.4. Przykład

Przykładowy graf z wagami krawędzi i wartościami heurystyki oraz przebieg algorytmu krok po kroku znajdziesz w prezentacji `materialy/Grafy.pdf` (slajdy 21–28). W tej prezentacji tablice `g` i `f` są inicjalizowane nieskończonością dla wszystkich wierzchołków — w naszej implementacji nie robimy tego, ale przebieg algorytmu jest ten sam. Warto samodzielnie rozpisać go na kartce, zanim zaczniesz kodować.

## 5. Mapa w ROS 2 — jak dostać się do komórek

### 5.1. Struktura `nav_msgs/msg/OccupancyGrid`

```
header                         # frame_id: "map"
info:
  resolution                   # rozmiar jednej komórki w metrach [m/komórkę]
  width                        # liczba komórek w poziomie (oś x)
  height                       # liczba komórek w pionie (oś y)
  origin                       # pozycja (geometry_msgs/Pose) lewego dolnego rogu komórki (0, 0) w układzie mapy
data                           # tablica int8 o długości width * height
```

Komórki są ułożone **wierszami**: komórka o indeksach $(ix, iy)$ (kolumna, wiersz) leży pod indeksem

$$\text{idx} = iy \cdot \text{width} + ix$$

Wartości w `data`:

| Wartość | Znaczenie |
|---------|-----------|
| `-1` | obszar nieznany |
| `0` | wolne |
| `100` | zajęte |
| `1…99` | prawdopodobieństwo zajętości (w `/map_cost` — koszt) |

### 5.2. Wskazówki dotyczące odczytu

W C++ pola wiadomości odczytujesz tak jak pola struktury, np. `map_dilated->info.width`, `map_dilated->info.origin.position.x`, `map_dilated->data[idx]`. Zwróć uwagę, że w klasie mapy są przechowywane jako `OccupancyGrid::UniquePtr` — **mogą być puste** (`nullptr`), jeżeli mapa jeszcze nie dotarła. Sprawdź to na początku metody `a_star` i wypisz ostrzeżenie (`RCLCPP_WARN`).

### 5.3. Konwersje współrzędnych

Przeliczenie punktu $(x, y)$ w metrach (układ `map`) na indeksy komórki oraz z powrotem (środek komórki):

$$ix = \left\lfloor \frac{x - o_x}{r} \right\rfloor, \qquad iy = \left\lfloor \frac{y - o_y}{r} \right\rfloor$$

$$x = o_x + (ix + 0{,}5)\, r, \qquad y = o_y + (iy + 0{,}5)\, r$$

gdzie $(o_x, o_y)$ to `info.origin.position`, a $r$ to `info.resolution`. Mapy w tym ćwiczeniu nie są obrócone (orientacja `origin` jest jednostkowa), więc obrót możesz pominąć.

Przy dzieleniu pamiętaj o rzutowaniu na `double`. Przed użyciem indeksów sprawdź, czy mieszczą się w zakresie $[0, \text{width})$ i $[0, \text{height})$ — punkt wskazany przez użytkownika może leżeć poza mapą.

### 5.4. Którą mapę wykorzystać

| Mapa | Zastosowanie |
|------|--------------|
| `/map_dilated` | **sprawdzanie, czy komórka jest dostępna**. Przeszkody są pogrubione o promień robota (0,25 m), więc robota można traktować jak punkt. Komórkę uznaj za wolną, gdy `0 <= data < 65`; wartość `-1` (nieznane) i wartości `>= 65` traktuj jako przeszkodę. |
| `/map_cost` | koszt 0–100 rosnący w miarę zbliżania się do przeszkód (0 = daleko od przeszkód); wykorzystywany tylko w zadaniu bonusowym. |

Obie mapy mają ten sam rozmiar, rozdzielczość i origin jak `/map`.

### 5.5. Konstrukcja wyniku — `nav_msgs/msg/Path`

`Path` to nagłówek i wektor `poses` typu `PoseStamped`. Wymagania `path_follower`:

- trasa musi mieć co najmniej 2 punkty; **pierwszy punkt jest pomijany** (to pozycja startowa), robot jedzie do kolejnych,
- każdy punkt powinien mieć `header.frame_id = "map"`,
- każdy punkt musi mieć poprawną orientację — ustaw `pose.orientation.w = 1.0`. Domyślnie wszystkie składowe kwaternionu są zerowe, co jest nieprawidłowe i spowoduje błędy transformacji TF w `path_follower`,
- wypełniaj `pose.position.x/y` współrzędnymi środków komórek trasy w metrach.

Sugerowana kolejność punktów trasy: `start` (dokładna pozycja robota) → środki kolejnych komórek trasy → `end` (dokładny cel z `/goal_pose`).

Gdy trasa nie istnieje, wypisz w logu komunikat **„Brak"** (`RCLCPP_WARN`) i **nie publikuj** żadnej trasy.

## 6. Zadania

### Zadanie 1 — graf z mapy

Przygotuj funkcje pomocnicze (jako metody klasy lub funkcje statyczne), które będą budować graf „w locie":

1. konwersja współrzędnych w metrach na indeks komórki i z powrotem (punkt 5.3),
2. sprawdzenie, czy komórka mieści się w mapie i jest dostępna (punkt 5.4),
3. wyznaczenie dostępnych sąsiadów komórki wraz z wagami krawędzi (8-sąsiedztwo; $r$ i $r\sqrt{2}$),
4. heurystyka — odległość euklidesowa do celu (punkt 3.3).

Wskazówki i pułapki:

- Pozycja startowa robota może leżeć w „pogrubionym" obszarze (np. gdy robot stoi blisko ściany) — nie odrzucaj startu z tego powodu.
- Cel wskazany w przeszkodzie lub poza mapą oznacza „Brak".
- Dla ruchu po przekątnej rozważ zabronienie „przecinania rogów", gdy któraś z dwóch sąsiadujących komórek jest przeszkodą.
- Jako identyfikatora wierzchołka wygodnie używać indeksu `idx` z punktu 5.1 (jedna liczba całkowita jako klucz w słowniku).

### Zadanie 2 — A\* podstawowy

Zaimplementuj A\* według pseudokodu 4.1. Wynikiem ma być trasa opublikowana na `/path` lub komunikat „Brak". Kolejka priorytetowa: `std::priority_queue` z odpowiednim komparatorem (domyślnie zwraca największy element — potrzebujesz najmniejszego).

### Zadanie 3 — A\* zmodyfikowany

Rozszerz implementację o zbiór przejrzanych wierzchołków zgodnie z pseudokodem 4.2. Wykonaj porównanie obu wersji: wypisz w logu liczbę rozwiniętych wierzchołków i długość trasy dla tych samych punktów start–cel. Czy trasy są takie same? Czy liczba rozwiniętych wierzchołków się zmieniła?

### Zadanie bonusowe — mapa kosztów

Uwzględnij mapę `/map_cost`, aby trasa trzymała się z dala od ścian (a nie tylko omijała pogrubione przeszkody). Zwiększ wagę krawędzi proporcjonalnie do kosztu komórki, np.:

$$w(x, y) = d(x, y) \cdot \left(1 + k \cdot \frac{\text{cost}(y)}{100}\right), \quad k \ge 0$$

Dla $k \ge 0$ waga nie jest mniejsza niż odległość geometryczna, więc heurystyka euklidesowa pozostaje dopuszczalna i spójna. Porównaj trasy dla różnych $k$.

## 7. Testowanie

Przetestuj rozwiązanie w co najmniej kilku scenariuszach (świat wybierasz parametrem `world:=` w `simulation.launch.py`):

| Scenariusz | Oczekiwany rezultat |
|------------|---------------------|
| cel w tym samym pomieszczeniu, bez przeszkód | trasa zbliżona do linii prostej |
| `hallway`: cel za zielonym blokiem | trasa omija blok, nie przechodzi przez ścianę |
| `cave`, `lines` | trasa przez wąskie przejścia, robot nie zderza się ze ścianami |
| cel w ścianie lub poza mapą | komunikat „Brak", brak nowej trasy |
| cel w obszarze niedostępnym (odseparowanym) | komunikat „Brak" |
| start i cel blisko siebie | poprawna, krótka trasa (min. 2 punkty) |
| kilka celów po sobie | za każdym razem poprawna nowa trasa |

Wskazówki dotyczące debugowania:

- Wypisuj w logu (`RCLCPP_INFO`) współrzędne i indeksy startu oraz celu, rozmiar mapy i liczbę rozwiniętych wierzchołków.
- Wyświetl w RViz2 `/map_dilated`, żeby zobaczyć, gdzie algorytm może przechodzić (punkt 4.9 w `01_wprowadzenie_ros2.md`).
- Najpierw sprawdź konwersje współrzędnych na prostych przykładach — typowe błędy to zamienione osie `x`/`y`, pominięty `origin` i indeks `iy * width + ix` zapisany jako `ix * height + iy`.
- Dla małych map (`hallway`, rozdzielczość 0,1 m) A\* działa natychmiast. Mapy `cave` i `lines` mają rozdzielczość 0,032 m (ok. 250 tys. komórek) — źle napisana implementacja (np. liniowe szukanie minimum zamiast kolejki priorytetowej) będzie wyraźnie wolna.
- Dla `h ≡ 0` otrzymasz algorytm Dijkstry — możesz porównać liczbę rozwiniętych wierzchołków z wersją z heurystyką euklidesową.
- Jeżeli robot jedzie nieregularnie po trasie z gęsto rozmieszczonymi punktami (w `cave` odległość między komórkami to 3 cm, a `path_follower` uznaje punkt za osiągnięty w promieniu 5 cm), możesz opublikować co kilka punktów trasy, pamiętając o zachowaniu ostatniego.

## 8. Przydatne odnośniki

- Dokumentacja ROS 2: <https://docs.ros.org/en/lyrical/ROS-Framework.html>
- Prezentacja z teorią grafów i A\*: `materialy/Grafy.pdf`
- A\* (Wikipedia, pełny pseudokod): <https://en.wikipedia.org/wiki/A*_search_algorithm>

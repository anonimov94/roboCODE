# roboCODE — instrukcja dla bota programującego

roboCODE to otwarte narzędzie do **symulacji i programowania robotów offline (OLP)**, ze szczególnym naciskiem na
**obróbkę robotem (frezowanie 3/5-osiowe z G-code/APT)** oraz **układy kooperatywne dwóch ramion**
(jedno ramię trzyma detal, drugie wrzeciono).

Zakres funkcji jest wzorowany na publicznie opisanych możliwościach komercyjnych programów OLP
(dokumentacja, tutoriale). **GUI i kod piszemy od zera.**

---

## 0. Zasady nadrzędne (czytaj zanim napiszesz linijkę kodu)

1. **Zero kodu, ikon, plików i formatów skopiowanych z cudzego oprogramowania.** Nie dekompilujemy, nie
   kopiujemy ikon, logo, nazw produktów ani układu graficznego 1:1. Odtwarzamy *funkcje*, a nie *wygląd*.
   Własne ikony (np. Lucide/Material Symbols na licencji open source), własna kolorystyka, własne nazwy.
2. **Modele robotów** bierzemy tylko z legalnych źródeł: pakiety ROS-Industrial (URDF), strony producentów,
   własne pomiary DH. Każdy model ma pole `source` i `license` w pliku `.robot.json`.
3. **Backend nie zależy od GUI.** Cała logika działa headless (CLI + testy). GUI to cienka warstwa
   wywołująca komendy z `robocode.core.commands`.
4. **Każda operacja użytkownika = komenda z undo/redo.** Żadnych bezpośrednich mutacji sceny z GUI.
5. **Bezpieczeństwo przede wszystkim.** Nic nie wysyłamy do prawdziwego robota bez: symulacji,
   sprawdzenia kolizji, limitów osi, prędkości oraz jawnego potwierdzenia użytkownika.
6. Jednostki wewnętrzne: **mm, stopnie** w API użytkownika; **mm, radiany** wewnątrz obliczeń.
   Pozy jako macierze 4x4 (`numpy`).
7. Każdy nowy moduł = testy `pytest`. PR bez testów nie przechodzi.

---

## 1. Architektura

```
robocode/
  core/
    pose.py          # macierze 4x4, konwencje kątów Eulera (XYZ, ZYX, ZY'X'', kwaterniony)
    scene.py         # Station / Item / Frame / Robot / Tool / Object / Target / Program / Curve
    commands.py      # Command + UndoStack (undo/redo, grupowanie podobnych akcji)
    io.py            # zapis/odczyt stacji (.rcs.json), import CSV krzywych/punktów
  kinematics/
    robot_model.py   # model DH 6-osiowy, FK, IK numeryczne, limity osi, konfiguracje
    configs.py       # flagi konfiguracji (Front/Rear, Up/Down, Flip/NoFlip)
  program/
    instructions.py  # MoveJ, MoveL, MoveC, SetFrame, SetTool, SetSpeed, SetRounding,
                     # Pause, Message, Call, SetIO/WaitIO, SimEvent
    simulator.py     # interpolacja ruchów, oś czasu, cykl, ślad (trace)
  machining/
    nc_parser.py     # G-code (G0/G1/G2/G3, A/B/C lub IJK), później APT
    toolpath.py      # ścieżka = lista (pozycja, normalna, posuw, zdarzenia)
    solver.py        # ścieżka -> cele robota (path-to-tool offset, optymalizacja obrotu wokół Z)
    events.py        # zdarzenia programu: zmiana narzędzia, obroty wrzeciona, M-kody, ekstruder
  coop/
    dual_arm.py      # podział ruchu: robot A trzyma detal, robot B wrzeciono
    calibration.py   # kalibracja baz (transformacja robot A <-> robot B z punktów wspólnych)
  post/
    base.py          # PostProcessor (API zgodne z ideą: MoveJ/MoveL/setFrame/setTool/RunCode)
    kuka_krl.py      # KUKA KRL (.src/.dat)
    abb_rapid.py     # ABB RAPID (.mod)
    generic_gcode.py # odwrotnie: robot -> G-code (debug)
  collision/
    checker.py       # (TODO) siatki + BVH (python-fcl lub trimesh), mapa kolizji
  drivers/
    base.py          # (TODO) sterownik online: connect, ping, move, run on robot
  gui/
    actions.py       # DEKLARATYWNA definicja menu, pasków narzędzi, skrótów (bez Qt!)
    main_window.py   # PySide6 – buduje GUI z actions.py
    viewport.py      # (TODO) widok 3D (pyvista/vtk lub moderngl)
    panels/          # panele: robot, narzędzie, układ, cel, projekt obróbki
  cli.py             # robocode run / simulate / generate
tests/
```

Przepływ danych:

```
CAD/CAM (G-code/APT/CSV) ─► machining.nc_parser ─► Toolpath
Toolpath + Station(robot, narzędzie, układ) ─► machining.solver ─► Program (MoveL...)
Program ─► program.simulator (podgląd, kolizje, czas cyklu)
Program ─► post.<marka> ─► plik dla kontrolera (KRL/RAPID/…)
```

---

## 2. Specyfikacja funkcji (backend)

### 2.1 Stacja i drzewo elementów
- Stacja = drzewo elementów. Typy: `Station`, `Frame` (układ odniesienia), `Robot`, `Tool` (TCP),
  `Object` (geometria STL/STEP), `Target`, `Program`, `MachiningProject`, `Curve`, `Points`.
- Każdy element ma: `name`, `parent`, `pose` (względem rodzica), `visible`, `children`.
- Przeciągnięcie elementu na inny = zmiana rodzica z zachowaniem pozycji absolutnej (opcja: bez zachowania).
- Wiele stacji otwartych jednocześnie.
- Aktywny układ i aktywne narzędzie per robot (używane przy uczeniu celów i nowych programach).
- Operacje: dodaj, usuń, zmień nazwę (F2), pokaż/ukryj (F7), kopiuj/wytnij/wklej (z dziećmi), scal obiekty.

### 2.2 Pozy i orientacje
- Reprezentacje: XYZ + Euler w różnych kolejnościach (XYZ statyczne, ZY'X'' – KUKA, XY'Z'' itd.),
  XYZ + kwaternion (ABB), macierz 4x4.
- Każdy robot ma domyślną konwencję zgodną z kontrolerem; post-procesor sam przelicza.
- Funkcje: odwróć pozę, wyzeruj (jednostkowa), kopiuj/wklej jako tablica lub macierz.

### 2.3 Robot
- Model: kinematyka DH (6 osi) + opcjonalne osie zewnętrzne (szyna, obrotnica, pozycjoner).
- FK, IK (analityczne dla typowych 6R z nadgarstkiem sferycznym; numeryczne jako fallback).
- Lista wszystkich rozwiązań IK (do 8 konfiguracji dla 6R) z flagami F/R, U/D, F/N.
- Limity osi (sprzętowe + programowe), prędkości i przyspieszenia osi.
- Pozycja domowa, przestrzeń robocza (podgląd).
- Parametry kinematyczne edytowalne (np. po kalibracji).

### 2.4 Narzędzie (TCP)
- TCP względem kołnierza lub względem innego TCP (np. frez = wrzeciono + offset Z).
- Geometria narzędzia do wizualizacji i kolizji, skala, przesunięcie geometrii bez zmiany TCP.
- Kalibracja TCP z N konfiguracji przegubów dotykających jednego punktu (metoda najmniejszych kwadratów).

### 2.5 Układy odniesienia
- Zagnieżdżanie układów (stół -> detal).
- Kalibracja układu: metoda 3 punktów (początek, punkt na osi X, punkt w płaszczyźnie XY)
  oraz z N punktów (dopasowanie sztywne, algorytm Kabscha).

### 2.6 Cele (targets)
- Cel kartezjański (zachowuje pozę względem układu) lub przegubowy (zachowuje wartości osi).
- „Naucz cel” (Ctrl+T) z bieżącej pozycji robota do aktywnego układu i narzędzia.
- „Cele na powierzchni”: klik na geometrii -> cel z normalną.
- Przesuń robota do celu, pokaż inne konfiguracje.

### 2.7 Programy i instrukcje
Instrukcje:
`MoveJ`, `MoveL`, `MoveC` (przez 2 cele), `SetFrame`, `SetTool`, `SetSpeed` (liniowa/przegubowa,
przyspieszenia), `SetRounding` (−1 = ruch dokładny), `Pause` (−1 = stop do wznowienia), `Message`,
`Call`/`InsertCode`/`StartThread`, `SetIO`/`WaitIO` (cyfrowe/analogowe, timeout),
`SimEvent` (dołącz/odłącz obiekt, pokaż/ukryj, ustaw pozycję abs./wzgl., robot do domu,
przesuń przenośnik, utwórz/usuń obiekt, czekaj na obiekt).

Zachowania:
- Pierwszy ruch programu dodaje automatycznie `SetFrame` + `SetTool`.
- Przeciąganie instrukcji (zmiana kolejności, przenoszenie między programami).
- Zmiana MoveJ <-> MoveL jednym kliknięciem.
- Walidacja MoveL: osobliwości (np. oś 5 ≈ 0°), limity osi, zmiana konfiguracji -> ostrzeżenie
  z propozycją MoveJ.
- Programy w Pythonie (makra) korzystające z API roboCODE.

### 2.8 Symulacja
- Uruchom (dwuklik programu), pauza, stop, krok po instrukcji, „start od tej instrukcji”.
- Pasek osi czasu (przewijanie w przód i w tył).
- Współczynnik prędkości symulacji (domyślnie 5x, szybki tryb 100x).
- Ślad TCP (trace), czas cyklu, sprawdzanie kolizji w trakcie (obiekty w kolizji na czerwono).

### 2.9 Generowanie programu (post-procesory)
- Generuj (F6), generuj jako… (Shift+F6), wyślij do robota (Ctrl+F6, FTP), wybór post-procesora per robot.
- Opcje: dzielenie długich programów (max linii na plik), łuki -> odcinki liniowe (max długość łuku),
  minimalny krok, wstawianie podprogramów inline, folder wyjściowy.
- Post-procesor to klasa Pythona z metodami: `ProgStart`, `ProgFinish`, `MoveJ`, `MoveL`, `MoveC`,
  `setFrame`, `setTool`, `setSpeed`, `setZoneData`, `Pause`, `RunCode`, `RunMessage`, `setDO`, `waitDI`,
  `ProgSave`.

### 2.10 Projekt obróbki robotem (najważniejsze dla nas)
Wejście ścieżki:
- plik NC (G-code, później APT/NCI), krzywe (z krawędzi geometrii lub CSV `x,y,z[,i,j,k]`),
  punkty (CSV/TXT), obiekt do druku 3D (zewnętrzny slicer -> G-code).

Ustawienia projektu:
- robot, układ, narzędzie; widoczność ścieżki (zielona) i normalnych.
- **Zdarzenia programu**: zmiana narzędzia (`SetTool(%1)`), obroty wrzeciona, M-kody -> `M_RunCode(n)`,
  akcje na start/koniec, wartość zaokrąglenia, ekstruder `E` -> `Extruder(E)` lub wyjście analogowe.
- **Najazd/odjazd**: domyślnie 100 mm wzdłuż normalnej, łańcuch wielu ruchów (wzdłuż osi, współrzędne, łuk).
- **Path-to-tool offset**: `rotz(θ)` + algorytmy: *minimalna zmiana orientacji* (frezowanie),
  *orientacja podąża za ścieżką* (cięcie), *robot trzyma obiekt* (remote TCP).
- **Optymalizacja**: dozwolony obrót wokół Z narzędzia (domyślnie ±180°, krok 20°), podgląd punktów
  osiągalnych (zielone) / nieosiągalnych (czerwone), auto-aktualizacja przy przesunięciu układu.
- **Preferowana konfiguracja startowa** (lista rozwiązań IK dla pierwszego punktu, „nawinięcie” osi 6).
- **Optymalizacja osi zewnętrznych** (priorytet: minimalny ruch szyny).
- Przycisk **Aktualizuj** -> zielony ✓ albo czerwony ✗ z komunikatem, **Symuluj**.

### 2.11 Tryb kooperatywny dwóch ramion (wyróżnik roboCODE)
- Robot A trzyma detal (remote TCP), robot B wrzeciono.
- Wejście: ścieżka 5-osiowa (pozycja + wektor narzędzia względem detalu).
- Solver dzieli ruch: A orientuje detal tak, by punkt obróbki był w strefie najwyższej sztywności
  i manipulowalności robota B; B wykonuje ruch względny.
- Kryteria kosztu: odległość od osobliwości (manipulowalność), limity osi, ciągłość przegubów,
  odległość od strefy preferowanej, kolizje A-B.
- Kalibracja baz A<->B z ≥3 wspólnych punktów (Kabsch).
- Eksport: dwa zsynchronizowane programy (znaczniki synchronizacji co N punktów) albo
  jeden program dla sterowania skoordynowanego producenta.

### 2.12 Narzędzia dodatkowe (kolejne wersje)
Pomiar punktów 3D, zmiana koloru, mapa kolizji, tworzenie/edycja krzywych z geometrii,
eksport symulacji do HTML 3D, kalibracja robota, strefy bezpieczeństwa, monitor I/O, pad do jogowania.

---

## 3. Specyfikacja GUI

GUI definiujemy **deklaratywnie** w `robocode/gui/actions.py` (id, etykieta PL/EN, skrót, ikona,
komenda backendu). `main_window.py` tylko to renderuje. Dzięki temu testy sprawdzają kompletność
GUI bez uruchamiania Qt.

### 3.1 Układ okna
- **Menu główne** (góra), **pasek narzędzi** (pod menu, z opcją etykiet tekstowych),
  **drzewo stacji** (lewy panel, zmiana szerokości Ctrl+kółko), **widok 3D** (środek),
  **panel właściwości** (prawy, dokowalny), **pasek stanu** (dół, podpowiedzi),
  **pasek symulacji** (dół, pojawia się przy symulacji).
- Zaznaczenie w drzewie = zaznaczenie w 3D i odwrotnie. Dwuklik = okno właściwości
  (wyjątek: dwuklik celu przesuwa robota, dwuklik programu uruchamia symulację).

### 3.2 Menu
| Menu | Pozycje |
|---|---|
| Plik | Nowa stacja, Otwórz…, Biblioteka robotów, Ostatnie (Ctrl+1), Zapisz (Ctrl+S), Zapisz jako…, Eksport symulacji (HTML), Zamknij |
| Edycja | Cofnij (Ctrl+Z), Ponów (Ctrl+Y), Historia, Wytnij, Kopiuj, Wklej, Usuń, Zmień nazwę (F2) |
| Program | Dodaj układ, Dodaj puste narzędzie, Naucz cel (Ctrl+T), Cele na powierzchni (Ctrl+Shift+T), Dodaj program, Dodaj makro Python, MoveJ, MoveL, MoveC, SetFrame, SetTool, SetSpeed, SetRounding, Pause, Message, Call, Set/Wait IO, Zdarzenie symulacji, Generuj (F6), Generuj jako… (Shift+F6), Wyślij do robota (Ctrl+F6), Szybka symulacja, Pauza, Edytuj post-procesory |
| Widok | Dopasuj wszystko (Alt+0), Izometryczny, Góra/Przód/Bok, Obrót/Przesuń/Zoom, Wyrównaj obrót, Pokaż/ukryj tekst (/), Układy większe (+)/mniejsze (−), Przestrzeń robocza (*), Pokaż/ukryj (F7) |
| Narzędzia | Ślad (trace), Sprawdzaj kolizje, Mapa kolizji, Kolor, Pomiar, Twórz krzywe, Edytuj krzywe, Język, Układ paska narzędzi, Opcje… |
| Narzędzia CNC | Kalibruj TCP, Kalibruj układ, Synchronizuj osie zewnętrzne, Projekt obróbki (Ctrl+M), Projekt podążania za krzywą, Projekt podążania za punktami, Projekt druku 3D, Importuj krzywą/punkty, **Projekt kooperatywny 2 ramion** |
| Połącz | Połącz z robotem (IP, ping, FTP), Rozłącz, Uruchamiaj na robocie |
| Pomoc | Dokumentacja (F1, kontekstowa), Skróty, O programie |

### 3.3 Pasek narzędzi (domyślny)
Otwórz · Biblioteka · Zapisz · Cofnij · Ponów · Dodaj układ · Naucz cel · Dopasuj wszystko · Izometryczny ·
Przesuń układ (Alt) · Przesuń TCP (Alt+Shift) · Kolizje (przełącznik) · Szybka symulacja · Pauza ·
Dodaj program · Dodaj makro · MoveJ · MoveL · Eksport symulacji.

### 3.4 Nawigacja 3D
LPM: zaznacz (Ctrl: wielokrotne) · ŚPM: przesuń (Ctrl+Alt+LPM) · PPM: obrót (Ctrl+Shift+LPM) ·
kółko: zoom (Shift+LPM). Alt + przeciąganie: przesuwanie układu; Alt+Shift: przesuwanie TCP
(gizmo z osiami i pierścieniami obrotu). Menu kontekstowe w 3D z tymi samymi akcjami (laptop bez myszy).
Tryby nawigacji do wyboru w Opcjach (zgodne z popularnymi CAD).

### 3.5 Panele
- **Panel robota**: jog osi (suwaki + pola, kółko myszy zmienia wartość), limity (dwuklik = edycja),
  kopiuj/wklej osie, TCP względem kołnierza, układ względem bazy, TCP względem układu (edycja = ruch),
  inne konfiguracje, Parametry (DH, post-procesor).
- **Narzędzie**: poza TCP, rodzic (kołnierz/inne TCP), „Więcej opcji” (skala, przesunięcie geometrii).
- **Układ**: poza względem rodzica, wybór formatu orientacji, przyciski kopiuj/wklej/odwróć/zeruj.
- **Cel**: poza + osie, kartezjański/przegubowy, przesuń do celu, konfiguracje.
- **Projekt obróbki** (dokowalny): wybór robota/układu/narzędzia, wejście ścieżki, zdarzenia,
  najazd/odjazd (Dodaj/Usuń), path-to-tool offset z podglądem i kółkiem myszy, algorytm,
  optymalizacja (±kąt, krok), konfiguracja startowa, przyciski Aktualizuj/Symuluj, status ✓/✗.
- **Opcje**: Ogólne, Wyświetlanie, Ruch (prędkości symulacji), Program (dzielenie, łuki, inline,
  folder), CAM (domyślne zdarzenia, najazd), CAD (dokładność importu), Python.

### 3.6 Skróty (komplet)
F1 pomoc · F2 nazwa · F3 opcje celu · F6 generuj · Shift+F6 generuj jako · Ctrl+F6 wyślij ·
F7 pokaż/ukryj · Alt+0 dopasuj · Ctrl+1 ostatni plik · Ctrl+T naucz cel · Ctrl+Shift+T cele na powierzchni ·
Ctrl+M projekt obróbki · Spacja szybka symulacja · Backspace pauza · Esc stop / wyjście z trybu wyboru ·
`/` tekst · `+`/`−` rozmiar układów · `*` przestrzeń robocza.

---

## 4. Kolejność prac (milestones)

1. **M1 – rdzeń headless** ✅ (szkielet w repo): pozy, scena, undo, FK/IK DH, instrukcje,
   parser G-code, solver obróbki z optymalizacją obrotu Z, post KRL/RAPID, zapis stacji, CLI, testy.
2. **M2 – GUI szkielet**: okno PySide6 z menu/paskiem z `actions.py`, drzewo stacji, panel właściwości.
3. **M3 – widok 3D**: pyvista/VTK, siatki robotów (STL z URDF), gizmo, ślad, ścieżka zielona/czerwona.
4. **M4 – symulacja**: oś czasu, prędkości, pauza/krok, czas cyklu.
5. **M5 – kolizje**: python-fcl, mapa kolizji, kolory.
6. **M6 – projekt obróbki w GUI**: pełny panel, podgląd osiągalności na żywo.
7. **M7 – kooperacja 2 ramion**: solver, kalibracja baz, eksport zsynchronizowany.
8. **M8 – sterowniki online**: najpierw symulowany sterownik, potem KUKA RSI/EKI, ABB EGM (opcjonalnie).
9. **M9 – import modeli**: URDF (ROS-Industrial) -> `.robot.json` + siatki.

## 5. Konwencje kodu
- Python ≥ 3.10, `numpy`, typowanie, `ruff`, `pytest`. GUI: `PySide6` (LGPL). 3D: `pyvista`.
- Funkcje obliczeniowe czyste (bez stanu globalnego). Scena zmieniana tylko przez `Command`.
- Komunikaty błędów zrozumiałe dla operatora („Oś 5 blisko osobliwości w punkcie 132 – użyj MoveJ
  albo zmień obrót wokół Z”).
- Każdy post-procesor ma test „złotego pliku” (golden file).

## 6. Uruchomienie
```bash
pip install -e .[dev]
pytest
python -m robocode.cli demo          # demo: G-code -> robot -> KRL
python -m robocode.cli machine examples/square.nc --post kuka --out out/
```

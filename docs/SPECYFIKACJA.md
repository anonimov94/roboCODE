# roboCODE — specyfikacja projektu (dokument nadrzędny)

> **Ten plik jest jedynym obowiązującym źródłem wymagań.** Jeśli README, komentarz w kodzie albo
> notatka bota mówi coś innego, obowiązuje ten dokument. Zmiana wymagań = zmiana tego pliku w tym samym
> commicie co kod.

| | |
|---|---|
| Wersja dokumentu | 1.0 (15.10.2026) |
| Wersja kodu | 0.2.0.dev0 (proof of concept: jeden robot, pełny przepływ) |
| Źródła scalone | `README.md` (pierwotna instrukcja), `chatgpt.txt`, `gemini.txt`, `roboCODE_roadmap_GROK.txt` |

---

## 1. Cel i zakres

roboCODE to otwarte narzędzie do **symulacji i programowania robotów offline (OLP)** z naciskiem na
**obróbkę robotem** (frezowanie 3/5-osiowe z G-code) oraz — jako wyróżnik w dalszej kolejności —
**frezowanie kooperatywne dwoma ramionami** (jedno trzyma detal, drugie wrzeciono; np. pianka/XPS).

Najważniejszy rezultat: **z pliku CAM powstaje program na prawdziwy kontroler (KUKA KRL, ABB RAPID),
który został sprawdzony na całej trajektorii, ma raport walidacji i daje się odtworzyć.**

Poza zakresem do odwołania: certyfikowane funkcje bezpieczeństwa, sterowanie w czasie rzeczywistym,
systemy embedded/mikrokontrolery (uwaga z notatki Gemini o ESP32/RPi nie dotyczy tego projektu).

---

## 2. Zasady nadrzędne

1. **Bezpieczeństwo ponad wygodę.** Błąd OLP to nie wyjątek w programie, tylko potencjalnie
   niebezpieczna trajektoria. Brak danych, nieobsługiwana instrukcja lub niepewny wynik →
   **jawny błąd albo ostrzeżenie, nigdy cichy „poprawny” program.**
2. **Eksport tylko po walidacji.** Plik dla kontrolera powstaje wyłącznie przez
   `robocode.pipeline.export_program()` (CLI i GUI F6 używają tej samej funkcji). Błąd krytyczny =
   brak pliku.
3. **Odtwarzalność.** Każdy przebieg zapisuje `<nazwa>.manifest.json`: wersja, commit, model robota
   (hash), narzędzie, układ, post-procesor, ustawienia, tolerancje, raport walidacji, sumy SHA-256
   wejścia i wyjścia. Zmiana któregokolwiek z tych elementów unieważnia poprzedni raport.
4. **Backend przed GUI.** Cała logika działa headless (CLI + testy). GUI to cienka warstwa wywołująca
   backend; nie mutuje sceny bezpośrednio (komendy z undo/redo).
5. **Najpierw jeden robot, cały przepływ.** Import ścieżki → kinematyka → walidacja → symulacja →
   eksport. Dwa ramiona dopiero po ustabilizowaniu tego przepływu.
6. **Zero cudzego kodu, ikon, formatów i wyglądu.** Odtwarzamy funkcje, nie wygląd. Modele robotów
   tylko z legalnych źródeł; każdy `.robot.json` ma pola `source` i `license`.
7. **Każda funkcja = testy.** Zmiana w FK/IK, parserze, interpolacji lub post-procesorze uruchamia
   testy regresyjne. PR bez testów nie przechodzi.
8. **Symulator nie zastępuje bezpieczeństwa stanowiska.** Patrz §12.

---

## 3. Rejestr decyzji (rozstrzygnięte sprzeczności między źródłami)

| # | Sprzeczność | Decyzja | Uzasadnienie |
|---|---|---|---|
| D1 | README: GUI i widok 3D jako M2–M3 (wcześnie). ChatGPT i Grok: GUI dopiero po działającym backendzie. | **Pierwotna decyzja o zamrożeniu pozostaje historyczna; GUI jest częściowo odblokowane** dla ograniczonego workflow opisanego w §10.1. Nie oznacza to gotowości produkcyjnej ani zgody na sterowanie robotem. | Backend i walidator są rozwinięte; dalsze GUI wdrażane etapami, z zachowaniem bramki walidacji i testu T1 dla uruchomienia na sprzęcie. |
| D2 | Kolejność wersji: README M1–M9 vs Grok v0.1–v1.0. | **Obowiązuje numeracja Groka (v0.x)**; milestones README zmapowane w §10. | Wersje Groka mają kryteria ukończenia. |
| D3 | Gemini: dokumentacja techniczna po angielsku. README: wszystko po polsku. | **Dokumentacja projektu po polsku**; identyfikatory i docstringi **nowego** kodu po angielsku; komunikaty dla operatora po polsku (później i18n). | Właściciel pracuje po polsku; kod zostaje czytelny dla społeczności open source. |
| D4 | Gemini: walidacja przez Pydantic. | **Nie teraz.** `dataclasses` + jawne walidatory domenowe. | Mniej zależności; walidacja geometrii i tak jest domenowa. |
| D5 | README §2.3: IK analityczne + numeryczne fallback. Stan kodu: tylko numeryczne. | v0.2: numeryczne IK z jawnym statusem + wyszukiwanie konfiguracji (nadgarstek, obroty ±360°). **IK analityczne dla nadgarstka sferycznego = zadanie v0.2.** | Testy pokazały, że numeryczne IK gubi konfiguracje ramienia. |
| D6 | Wyróżnik projektu (dual-arm) vs rada „zacznij od jednego robota”. | Kod `coop/` zostaje jako **eksperymentalny**, bez rozwoju do v0.8. Przed nim osie zewnętrzne (v0.7). | Grok: osie zewnętrzne praktyczniejsze; ChatGPT: najpierw jeden robot. |
| D7 | Nieobsługiwane kody G-code: README milczy. ChatGPT: nie pomijać po cichu. | **Tryb ścisły domyślnie**: błąd z numerem linii. `--no-strict` zamienia je w ostrzeżenia (niezalecane). | Zasada 1. |
| D8 | Gemini: HAL, timeouty, fail-safe. | Przyjęte w warstwie `drivers/` (v1.0, opcjonalnie). Domyślnie offline, patrz §12. | Brak komunikacji z robotem w PoC. |

---

## 4. Konwencje (obowiązkowe)

| Temat | Konwencja |
|---|---|
| Jednostki API użytkownika | **mm, stopnie**, posuw **mm/min** (jak w CAM), prędkości robota **mm/s**, **°/s** |
| Jednostki wewnętrzne | mm; kąty w stopniach w strukturach danych, radiany tylko wewnątrz obliczeń |
| Pozy | macierze jednorodne 4×4 `numpy`, prawoskrętne, ortonormalne (`pose.is_valid_pose`) |
| Składanie | `T_A_C = T_A_B @ T_B_C` (poza B wyrażona w A), odwrotność `pose.inv` |
| Łańcuch robota | `TCP_w_bazie = FK(q) @ TCP_narzędzia`; cel w układzie detalu: `inv(T_baza_detal) @ FK(q) @ TCP` |
| Euler | `XYZ_static` (a=rx, b=ry, c=rz; R = Rz·Ry·Rx) oraz `KUKA_ABC` (A=rz, B=ry, C=rx; Z-Y'-X''). Kwaterniony ABB `[w,x,y,z]`. |
| Oś narzędzia | oś Z narzędzia; przy frezowaniu skierowana **przeciwnie** do wektora ścieżki (`tool_flip=True`) |
| Ścieżka NC | wektor narzędzia z A/B (B wokół Y, potem A wokół X) albo IJK w G0/G1 |
| Tolerancje | **osobne**, z jednostkami: zgodność celu 0,05 mm / 0,05°; weryfikacja pliku 0,01 mm / 0,01°; łuk NC 0,01 mm; próbkowanie MoveL 5 mm; MoveJ 2°; skok konfiguracji 30°; ostrzeżenie przy limicie 2°; osobliwość nadgarstka 5° |
| Liczby zmiennoprzecinkowe | nigdy porównanie `==`; zawsze tolerancja właściwa dla wielkości |
| Dane niepoprawne | NaN/inf, zerowe wektory, nieortonormalne obroty → wyjątek domenowy (`robocode.errors`) lub błąd w raporcie |

---

## 5. Architektura

```
robocode/
  errors.py            wyjątki domenowe (NCParseError z numerem linii, ExportBlockedError, …)
  pipeline.py          run_job(): NC → ścieżka → program → walidacja → post → weryfikacja → manifest
                       export_program(): JEDYNA droga zapisu pliku sterownika
  core/   pose.py      macierze 4×4, Euler, kwaterniony, Kabsch, is_valid_pose
          scene.py     drzewo stacji: Station/Frame/Robot/Tool/Object/Target/Program/Curve
          commands.py  Command + UndoStack
          io.py        zapis/odczyt stacji .rcs.json
          fileio.py    zapis atomowy, SHA-256
  kinematics/          model DH 6R: FK, Jakobian, IK (ik_solve ze statusem, ik_all), konfiguracje
  robots/              biblioteka modeli *.robot.json (source + license obowiązkowe)
  machining/           nc_parser (stan modalny), toolpath, solver (obrót Z, najazd/odjazd)
  program/             instructions, simulator (pełna trajektoria), report (Issue), validator
  post/                base (generator), kuka_krl, abb_rapid
  coop/                [eksperymentalne] dual-arm, kalibracje baz/TCP/układu
  tools/               report_html (raport HTML z manifestu), sample_nc (przykładowe programy NC)
  collision/ drivers/  [szkielety] v0.7 / v1.0
  gui/                 [częściowo odblokowane, D1] actions (deklaratywne), handlers, main_window, render, viewport
  cli.py               robocode demo | machine | report | samples | render | gui
```

Przepływ danych (PoC):

```
plik NC ──parse_gcode──► Toolpath (mm, wektory, posuw, zdarzenia, nr linii, ostrzeżenia)
Toolpath + robot/układ/narzędzie ──solve──► Program (MoveJ/MoveL/SetSpeed/Call/Pause, src_line)
Program ──validate_program──► ValidationReport (statyczna + cała trajektoria w symulatorze)
   └── błąd? ─► STOP: brak pliku, manifest z raportem
Program ──post.generate──► tekst KRL/RAPID w pamięci ──verify_post_output──► zgodność poz
   └── zgodne ─► zapis atomowy <nazwa>.src/.mod + <nazwa>.manifest.json
```

---

## 6. Kontrakty modułów

### 6.1 `core.pose`
- Wejście/wyjście: 4×4 `float`. `from_xyzabc`/`to_xyzabc` (konwencje z §4), `to_quaternion`/`from_quaternion`,
  `pose_distance` → (mm, °), `fit_rigid` (Kabsch, ≥3 niewspółliniowe punkty), `is_valid_pose`.
- Gwarancja: obustronna konwersja Euler/kwaternion odtwarza pozę (także przy B = ±90°) — testy.

### 6.2 Biblioteka robotów (`robocode.robots`)
- Plik `<nazwa>.robot.json`: `name, source, license, dh[a, alpha°, d, theta_offset°], limits_deg, joint_sign, vmax_deg_s`.
- `load_robot()` odrzuca model bez `source`/`license`.
- Stan: `generic6r_900` (model **syntetyczny**) oraz `ur5e` (geometria nominalna przeliczona
  z ROS-Industrial URDF i plików parametrów, licencja BSD-3-Clause). Model UR5e jest zgodny
  z domyślnymi transformacjami URDF, ale nie zastępuje kalibracji kinematycznej konkretnego egzemplarza
  przed użyciem na stanowisku.

### 6.3 Kinematyka
- `fk(q°) → 4×4` (kołnierz w bazie). `jacobian`, `manipulability`.
- `ik_solve(T, seed) → IKResult(q, status, message)`, status: `ok | no_convergence | joint_limits | invalid_input`.
  Nigdy „przypadkowa” pozycja.
- `ik_all(T)` → unikalne rozwiązania w limitach. Dla obsługiwanej geometrii DH 6R z nadgarstkiem
  sferycznym używa gałęziowego IK analitycznego (bark, łokieć, nadgarstek); pozostałe geometrie
  zachowują numeryczne wyszukiwanie ziarnami.
- Testy: FK→IK→FK na losowych konfiguracjach, unikalność rozwiązań, ciągłość małych kroków (brak skoków 360°).

### 6.4 Parser NC (obsługiwany dialekt)

| Grupa | Obsługiwane | Zachowanie |
|---|---|---|
| Ruch | G0, G1, G2, G3, G4 (P = sekundy) | łuki dzielone na odcinki `arc_seg_mm` |
| Płaszczyzna | G17 (XY), G18 (ZX), G19 (YZ) | łuki w każdej płaszczyźnie, ruch helikalny |
| Jednostki | G20 (cale), G21 (mm) | wszystko przeliczane na mm; brak G20/G21 → ostrzeżenie, przyjęte mm |
| Wymiarowanie | G90, G91 | XYZ i A/B; środki łuków I/J/K zawsze przyrostowe |
| Łuki | I/J/K lub R (R<0 = łuk > 180°) | niespójny promień, za małe R, pełny okrąg przez R → błąd |
| Orientacja | A/B albo IJK w G0/G1 | A/B + IJK naraz → błąd; zerowy wektor → błąd |
| Inne | F, S, T, M3/M4/M5/M6, M0/M1 → Pause, M2/M30 → koniec, inne M → `M_RunCode(n)`, E, N, O, komentarze `( )` `;`, `%` | kolejność w bloku: T/S/F przed M3 |
| Bez skutku | G40, G49, G54, G80, G94 | zgodne z naszą semantyką |
| **Odrzucane** | G41/G42 (korekcja promienia), G43 (długości), G81–G89 (cykle), G93, G28, G55–G59, oś C, inne | `UnsupportedCommandError` z numerem linii (tryb ścisły) |

- Wynik: `Toolpath` (punkty z numerem linii źródła, `warnings`, `source_units`, `end_events`).
- Test fuzzingu: 500 losowych wejść — tylko `NCParseError` albo poprawna ścieżka.

### 6.5 Solver obróbki
- Dla każdego punktu szuka obrotu wokół Z narzędzia (±`z_range_deg`, krok `z_step_deg`), wybiera
  rozwiązanie osiągalne, z dala od osobliwości i najbliższe poprzedniemu.
- Konfiguracje startowe pierwszego punktu można wyliczyć (`enumerate_start_configurations`) i wybrać
  przez 1-based `MachiningSettings.start_configuration_index`; CLI: `--list-start-configurations`,
  potem `--start-config N`. Lista porządkuje rozwiązania IK według odległości od `start_joints`
  (lub bieżących osi robota) i manipulowalności; błędny indeks blokuje eksport.
- Dla numerycznych modeli lista obejmuje gałęzie znalezione z lokalnego ziarna i jego wariantów
  nadgarstka — nie gwarantuje kompletności wszystkich rozwiązań matematycznych. Wybrany indeks,
  obrót Z i osie są zapisywane w manifeście.
- Najazd/odjazd wzdłuż osi narzędzia (`approach_mm`, domyślnie 100).
- Duże zmiany osi narzędzia w jednym bloku są zagęszczane (`max_orientation_step_deg`, domyślnie 5°;
  pozycja liniowo, wektor przez slerp) — tak jak sterownik interpoluje orientację w LIN.
- Komunikat rozróżnia „nieosiągalny” (brak rozwiązania IK) od „wymaga skoku konfiguracji” (`CONFIG_JUMP`).
- Instrukcje dostają `src_line` (linia NC) → raport wskazuje miejsce w pliku CAM.
- Nieosiągalne punkty → błąd z numerem linii i podpowiedzią.

### 6.6 Symulator
- MoveJ interpolowany w osiach (krok 2°), MoveL w przestrzeni kartezjańskiej (krok 5 mm **i** 2° orientacji,
  slerp) — także ruchy będące samym obrotem narzędzia.
- Sprawdza **każdą próbkę**: osiągalność, limity (+ ostrzeżenie 2° przed limitem), skok konfiguracji,
  osobliwości, prędkości osi względem `vmax_deg_s` (robot nie utrzyma posuwu → ostrzeżenie).
- Czas cyklu zakłada profil trapezowy (lub trójkątny przy krótkim ruchu), start i koniec każdego
  instrukcyjnego ruchu w spoczynku: MoveJ — maksimum czasów osiowych przy min(prędkość programowa,
  `vmax_deg_s`) i przyspieszeniu osiowym; MoveL/MoveC — dłuższy z czasu translacji ograniczonej
  przyspieszeniem liniowym oraz czasu obrotu TCP przy 90°/s. Orientacja nie ma osobnego profilu
  przyspieszenia, a zaokrąglenia/blending nie skracają oszacowania.
- Domyślne przyspieszenia symulatora (`SimTolerances`): 500 mm/s² i 300°/s². `SetSpeed.linear_acc`
  i `SetSpeed.joint_acc` nadpisują odpowiednie wartości dla kolejnych ruchów; wartości muszą być
  skończone i dodatnie. Raport INFO `ACCELERATION_ASSUMED` wymienia użyte założenia, gdy dane
  programu ich nie zastępują. To estymata, nie dane katalogowe; posty KRL/RAPID nie emitują tych
  przyspieszeń, a rzeczywisty czas zależy od sterownika, dynamiki robota, obciążenia i blendingu.
- MoveC symulowany jako prawdziwy łuk przez punkt pomocniczy (orientacja interpolowana jak `$ORI_TYPE=#VAR`);
  punkty współliniowe → `MOVEC_DEGENERATE`.
- Tolerancje (`SimTolerances`) muszą być skończone i > 0 — inaczej wyjątek, nie ciche wyłączenie kontroli.

### 6.7 Przybliżona kontrola kolizji
- Każda próbka symulacji może być sprawdzona przez `CollisionChecker`: odcinki między ramkami DH robota
  są traktowane jako kapsuły (domyślny promień 28 mm), a obiekty stacji z `box_size` jako prostopadłościany
  w ich lokalnym układzie. To uproszczone proxy, nie geometria CAD robota.
- Trafienie dodaje ostrzeżenie `COLLISION_PROXY` z numerem instrukcji i linią NC; samo nie blokuje eksportu,
  bo model może dawać fałszywe trafienia i pominięcia. Raport zawsze zawiera też `COLLISION_NOT_CHECKED`,
  jasno wskazujące niepełny zakres. Kontrola nie jest funkcją bezpieczeństwa ani zamiennikiem symulatora OEM.
- Ukryte obiekty są pomijane. Obiekty siatkowe bez `box_size`, samokolizje i strefy mocowań nie są
  sprawdzane; siatki wywołują osobne ostrzeżenie `COLLISION_MESH_NOT_CHECKED`. Domyślnie pomijane jest
  proxy narzędzia (promień 10 mm), aby uniknąć zgłoszenia planowanego kontaktu skrawającego.
- API `CollisionChecker(pairs=..., link_radius_mm=..., tool_radius_mm=...)` pozwala ograniczyć mapę
  do listy par `(nazwa_robota/komponent, nazwa_obiektu)`; jawne dopisanie komponentu `tool:<nazwa>`
  włącza jego proxy. Checker można przekazać do `validate_program`, `simulate`, `export_program` lub
  `run_job`. Efektywna metoda i parametry mapy są zapisane w manifeście eksportu.
- Nie ma jeszcze edytora mapy kolizji w GUI, testu kolizji siatka–siatka/FCL ani walidacji z geometrią
  konkretnego producenta. Wynik wymaga sprawdzenia w symulatorze producenta.

---

## 7. Walidacja przed eksportem (`program.validator`)

1. **Parser** — ostrzeżenia parsera przechodzą do raportu.
2. **Statyczna** — robot przypisany, są ruchy, SetFrame/SetTool przed pierwszym ruchem, poprawne macierze
   i wektory osi, osie w limitach, prędkość w zakresie [0,1; 1000] mm/s (wyłapuje pomyłki mm/min↔mm/s
   i cale), przyspieszenia `SetSpeed` skończone i > 0, zaokrąglenie w zakresie, **osie celu dają zapisaną
   pozę** (≤ 0,05 mm / 0,05°), lista podprogramów wymaganych na sterowniku.
3. **Trajektoria** — cały program w symulatorze (§6.6). Przy błędach danych symulacja jest pomijana z błędem
   `SIM_SKIPPED`. Dla próbek MoveL/MoveC raport INFO `CONTOUR_DEVIATION` podaje maksymalną i średnią
   odchyłkę pozycji TCP od punktu zadanego interpolatorowi [mm] oraz liczbę próbek; MoveJ jest pomijany.
   To metryka symulatora, nie certyfikat dokładności fizycznego robota. INFO `ACCELERATION_ASSUMED`
   ujawnia domyślne przyspieszenia użyte do szacowania cyklu.
4. **Post (odczyt zwrotny)** — wygenerowany tekst KRL/RAPID jest parsowany z powrotem: pozy porównywane
   z programem (≤ 0,01 mm / 0,01°), osie PTP/MoveAbsJ (≤ 0,001°). Wyłapuje błędy konwencji Eulera/kwaternionów
   i formatowania.
5. **Raport** — `Issue(severity, code, message, stage, instruction, source_line, hint)`;
   `ERROR` blokuje eksport, `WARNING` trafia do operatora i manifestu.

Kody błędów (stałe, do automatyzacji): `NC_PARSE, EMPTY, NO_ROBOT, MISSING_SETFRAME, MISSING_SETTOOL,
BAD_POSE, BAD_JOINTS, JOINT_LIMIT, SPEED_RANGE, JOINT_SPEED_RANGE, ROUNDING_RANGE, TARGET_MISMATCH,
UNREACHABLE, CONFIG_JUMP, MOVEC_DEGENERATE, SIM_SKIPPED, POST_COUNT, POST_MISMATCH, POST_JOINT_COUNT,
POST_JOINT_MISMATCH, STATION`. Informacja: `CONTOUR_DEVIATION`. Ostrzeżenia: `PARSER, NEAR_JOINT_LIMIT,
SINGULARITY, JOINT_SPEED, CONFIG_DIFFERS,
CONTROLLER_SUBPROGRAMS, COLLISION_NOT_CHECKED` (obecne, gdy pełna kontrola kolizji nie jest dostępna),
`TOOL_LOAD_DEFAULT` (ABB), `STALE_OUTPUT`.

**Czego walidacja jeszcze NIE robi:** pełnej kontroli kolizji (geometrii siatkowej, samokolizji i stref
mocowań), potwierdzania przyspieszeń na konkretnym robocie ani weryfikacji składni na prawdziwym
kontrolerze. Odchyłka konturu jest liczona dla próbek TCP ścieżek MoveL/MoveC, a nie względem
powierzchni CAD. Raport nie może być traktowany jako
dowód bezpieczeństwa.

---

## 8. Odtwarzalność i pliki wynikowe

- `<nazwa>.manifest.json` (format `robocode-manifest/1`) zapisywany **zawsze**, także przy blokadzie.
  Pola: `robocode_version, git_commit, created_utc, input{name, sha256, units, strict_parser},
  cell{robot{name, model_source, model_license, model_sha256, base_pose, start_joints}, frame, tool},
  post{name, class, version, tool_load}, settings, limits, validation, exported, outputs[{file, sha256}]`.
- Zapis atomowy (plik tymczasowy → `os.replace`): przerwany zapis nie zostawia pozornie poprawnego programu.
- Przy blokadzie: jeśli w folderze leży starszy plik o tej samej nazwie → ostrzeżenie `STALE_OUTPUT`.
- Format stacji `robocode-station/1` (`.rcs.json`); przyszłe zmiany formatu = numer wersji + migracja
  albo czytelny błąd.

---

## 9. Post-procesory

| Post | Plik | Orientacja | Stan |
|---|---|---|---|
| KUKA KRL | `.src` | A,B,C = Z,Y',X'' | odczyt zwrotny ✅; golden file (`tests/golden/controller_golden.src`) ✅; nagłówek `BAS(#VEL_PTP,20)`, `$ORI_TYPE=#VAR`, `$CIRC_TYPE=#BASE`; PTP dokładny (bez C_PTP); `BAS(#VEL_PTP,%)` w procentach vmax; obciążenie i dzielenie programu; składnia **do weryfikacji w WorkVisual/na sterowniku** |
| ABB RAPID | `.mod` | kwaternion | odczyt zwrotny ✅; golden file (`tests/golden/controller_golden.mod`) ✅; strefy: `fine`/predefiniowane `zN`/inline `zonedata` dla innych promieni; `robconf` z osi (cf1/cf4/cf6), `ConfL\Off`; obciążenie i dzielenie programu; składnia **do weryfikacji w RobotStudio/na sterowniku** |

**Mapowanie zdarzeń** (`--events plik.json`, przykłady `examples/events_kuka.json`, `events_abb.json`):
zdarzenie (`SpindleOn`, `SpindleOff`, `Spindle(n)`, `SetTool(n)`, `M_RunCode(n)`) → dowolny kod sterownika
(`{0}` = argument). Bez mapy zdarzenia są wywołaniami podprogramów — jeśli ich brak na sterowniku, program się
nie załaduje (bezpieczna awaria), a raport ostrzega `CONTROLLER_SUBPROGRAMS`. Mapa trafia do manifestu.
| generic | `.txt` | — | debug, bez weryfikacji |

Wymagania otwarte: mapowanie zdarzeń (`SpindleOn`, `Spindle(n)`, `SetTool(n)`, `M_RunCode(n)`) na podprogramy konkretnej celi,
deklaracja `$APO`/zone per ruch. **Test tekstowy nie zastępuje walidacji na docelowym kontrolerze.**

### 9.2 Dzielenie długich programów

- API `export_program`/`run_job` ma `max_lines_per_file`; CLI `robocode machine` przyjmuje
  `--max-lines-per-file` (domyślnie 5000, wartość dodatnia).
- Podział następuje wyłącznie na granicach instrukcji. Każdy plik ma własny nagłówek/procedurę,
  a przed dalszą częścią odtwarzany jest ostatni `SetFrame`, `SetTool`, prędkość i zaokrąglenie.
  Pliki dostają sufiksy `_001`, `_002` itd.; każdy jest osobno odczytywany zwrotnie, zapisany atomowo
  i wpisany do manifestu z sumą SHA-256.
- Raport ostrzega o podziale i o pozostawionych częściach z poprzedniego eksportu. Nie ma automatycznego
  wywołania następnej części ani gwarancji wznowienia stanu procesu/sterownika; operator musi ręcznie
  uruchamiać pliki w kolejności i zweryfikować bezpieczne przekazanie ruchu. Starych plików nie kasujemy.
- Limit mniejszy od nagłówka lub pojedynczej instrukcji blokuje eksport, zamiast emitować plik niezgodny.

### 9.1 Obciążenie narzędzia

- API `ToolLoad(mass_kg, cog_mm, inertia_kg_m2)` przyjmuje dodatnią masę [kg], 3 współrzędne środka
  ciężkości [mm] oraz 3 nieujemne składowe bezwładności [kg·m²]. Niepoprawne lub nieskończone dane
  odrzucane są przed generowaniem.
- CLI: `robocode machine ... --tool-load load.json`, gdzie JSON ma pola `mass_kg`, `cog_mm: [x,y,z]`
  i `inertia_kg_m2: [Ix,Iy,Iz]`. Wygenerowane ustawienie i parametry wejściowe są zapisane w manifeście.
- ABB: wartości trafiają do `tload` w `tooldata rcTool` (orientacja osi głównych bezwładności pozostaje
  jednostkowa). KUKA: `$LOAD` zawiera masę, pierwsze momenty `M*CoG` w [kg·m] i składowe bezwładności
  w [kg·m²].
- Bez konfiguracji ABB zachowuje dotychczasowe 1 kg / CoG Z=50 mm; KUKA nie ustawia `$LOAD`.
  Oba przypadki zgłaszają `TOOL_LOAD_DEFAULT`; są to wyłącznie wartości zastępcze, nie dane dla produkcji.
  Symulator nie używa tych parametrów do sprawdzenia dynamiki. Składnię i wartości trzeba zweryfikować
  w odpowiednim oprogramowaniu/sterowniku producenta przed użyciem.

---

## 10. Roadmapa

| Wersja | Nazwa | Zakres (skrót) | Kryterium ukończenia | Stan |
|---|---|---|---|---|
| v0.1 | Foundation | szkielet, pozy, scena, undo, CLI, testy | pytest przechodzi, zapis/odczyt `.rcs.json` | ✅ |
| v0.2 | Core | DH 6R, IK + konfiguracje, limity, TCP, układy, cele, MoveJ/L, biblioteka 1–2 robotów | stacja z kodu, naucz cel, MoveL, IK | 🟡 IK analityczne i model nominalny UR5e gotowe; brak katalogowych limitów dynamiki i pełnej listy konfiguracji |
| v0.3 | MVP Machining | parser NC, toolpath, solver (obrót Z, wybór konfiguracji startowej), najazd/odjazd, CLI | program z G-code otwiera się w oprogramowaniu producenta | 🟡 wybór konfiguracji z listy IK gotowy; brak weryfikacji u producenta |
| v0.4 | Simulation & Post | symulator z estymacją trapezoidalną czasu cyklu, KRL/RAPID, walidacja MoveL, odchyłka konturu, obciążenie narzędzia, dzielenie programów, **walidator centralny, manifest** | program z v0.3 symulowany, wyeksportowany i wgrany na robota | 🟡 walidator + manifest + golden files + metryka odchyłki + estymacja przyspieszeń + konfigurowalne obciążenie + dzielenie ✅; brak: test na robocie |
| v0.5 | Usable GUI | panele, viewport, Projekt obróbki w GUI, animacja symulacji, podgląd osiągalności | cały workflow bez CLI | 🟡 raport walidacji i podstawowa oś czasu symulacji gotowe; reszta workflow GUI nadal ograniczona bramką testu T1 |
| v0.6 | Advanced Machining | kalibracja TCP/układu, algorytmy orientacji, zdarzenia, CSV, MoveC, makra | kalibracja + dobry program 5-osiowy | 🟡 kalibracje TCP/baz/układu dostępne w CLI i GUI; algorytmy orientacji i makra otwarte |
| v0.7 | Robustness | kolizje (fcl/trimesh), osie zewnętrzne, mapa kolizji | sprawdzanie kolizji i limitów, pozycjoner | 🟡 proxy kapsuł–prostopadłościan i przełącznik GUI gotowe; brak siatek/FCL, samokolizji i edytora mapy |
| v0.8 | Dual-Arm | kalibracja baz A↔B, solver podziału ruchu, 2 zsynchronizowane programy | prosty przypadek kooperatywny działa | ⏳ (eksperymentalny kod) |
| v1.0 | Production | dokumentacja użytkownika, więcej postów, driver online (opcja), instalator | można polecić firmie (ze świadomością ograniczeń) | ⏳ |

Mapowanie starych milestones README: M1→v0.1–v0.3, M2/M3→v0.5, M4→v0.4, M5→v0.7, M6→v0.5/v0.6,
M7→v0.8, M8→v1.0, M9→v0.2 (modele robotów).

### 10.1 Najbliższe zadania (kolejność)
1. Weryfikacja importu i kompilacji KRL/RAPID w WorkVisual/RobotStudio — wynik wpisać do macierzy §11.
   Eksport demo i wewnętrzny read-back obu postów przechodzą; aplikacje vendor nie są dostępne
   w środowisku deweloperskim, więc ich zgodność składniowa pozostaje niepotwierdzona.
2. Test na prawdziwym robocie w trybie T1 i ze zredukowaną prędkością, dopiero po pozytywnej ocenie
   bezpieczeństwa stanowiska. Środowisko nie ma robota, sterownika OEM ani skonfigurowanej celi;
   nie wysłano poleceń ruchu. Potrzebne są zgodny model robota, zmierzone TCP/baza/obciążenie,
   program zatwierdzony przez kompilator vendor i uprawniony operator na stanowisku. Próba wyłącznie
   według procedur producenta/zakładu: T1, reduced speed, ręczne zezwolenie, krok po kroku i na sucho.
   Symulacja roboCODE nie jest testem T1 ani funkcją bezpieczeństwa.
3. Uzupełnienie katalogowych limitów prędkości/przyspieszeń osi i konfiguracji IK Rear/Down.
4. GUI v0.5: gizmo edycji w 3D; podgląd osiągalności jest dostępny w panelu projektu i jako
   zielone/czerwone punkty ścieżki po obliczeniu projektu.
5. Pełna kontrola kolizji v0.7: geometria siatkowa/FCL, samokolizje i edytor mapy kolizji.
6. v0.6: orientacja podążająca za ścieżką, remote TCP, makra Python i import krzywych geometrii.
7. Osie zewnętrzne; następnie kalibracja A↔B w przepływie dual-arm i dwa zsynchronizowane programy.
8. v1.0: dokumentacja użytkownika, kolejne posty, opcjonalny sterownik online i instalator.
9. Infrastruktura: wznowić CI po usunięciu blokady rozliczeń konta i rozstrzygnąć otwarty PR #1.
Wdrożone po pierwotnej decyzji D1 na żądanie użytkownika — ograniczone funkcje GUI offline, nie
sterowanie robotem:
- Panel Projektu obróbki prezentuje `ValidationReport`; oś czasu odtwarza próbki udanej symulacji,
  a suwak, pauza i Stop pozwalają sterować podglądem. Nie jest to reprezentacja rzeczywistego czasu
  cyklu ani test T1.
- Przełącznik kolizji obejmuje tylko przybliżone proxy opisane w §6.7.
- Pozostały workflow GUI, w tym edycja gizmem i podgląd osiągalności, nadal jest otwarty.

---

## 11. Macierz zgodności (funkcja → test → stan)

| Funkcja | Moduł | Test | Stan |
|---|---|---|---|
| Pozy, Euler (z gimbal lock), kwaterniony, Kabsch | core.pose | test_core, test_kinematics_props | ✅ gotowe |
| Scena, undo/redo, `.rcs.json` | core | test_core | ✅ gotowe |
| FK, IK ze statusem, konfiguracje (etykiety F/R, U/D, F/N dla analitycznych gałęzi 6R) | kinematics | test_kinematics_props | 🟡 etykiety gałęzi analitycznych pokryte testem; wyszukiwanie numeryczne pozostaje niekompletne |
| Parser NC: G0–G4, G17–G19, G20/21, G90/91, R/IJK, A/B/IJK, zdarzenia, tryb ścisły | machining.nc_parser | test_nc_parser (31 testów, w tym fuzzing) | ✅ gotowe dla opisanego dialektu |
| Import CSV krzywych | machining.toolpath | test_machining | ✅ |
| Solver obróbki + lista/wybór konfiguracji startowej z etykietą konfiguracji | machining.solver | test_machining, test_validator_pipeline, test_review_fixes | 🟡 etykiety rozróżniają gałęzie analityczne 6R; numeryczne IK nie gwarantuje kompletności listy |
| Symulacja pełnej trajektorii + estymacja czasu cyklu z przyspieszeniami | program.simulator | test_validator_pipeline, test_review_fixes | 🟡 założone przyspieszenia, brak profilu przyspieszeń orientacji/blendingu i weryfikacji na sterowniku |
| Walidator centralny + odczyt zwrotny posta | program.validator | test_validator_pipeline | ✅ gotowe (zakres §7) |
| Manifest, zapis atomowy, blokada eksportu i dzielenie programu z kontrolą limitu linii | pipeline, core.fileio, post.base | test_validator_pipeline, test_post_golden | ✅ gotowe; uruchomienie części wymaga nadzoru operatora |
| Raport HTML z manifestu (`robocode report`) | tools.report_html | test_tools (m.in. escapowanie HTML) | ✅ gotowe |
| Przykładowe programy NC (`robocode samples`) | tools.sample_nc | test_tools (każdy przez pełny pipeline) | ✅ gotowe |
| Post KUKA KRL | post.kuka_krl | test_machining, test_review_fixes, test_post_golden, read-back 42 pozy | 🟡 eksport i read-back lokalny OK; WorkVisual nieobecny, składnia niezweryfikowana kompilatorem vendor |
| Post ABB RAPID | post.abb_rapid | test_review_fixes, test_post_golden, read-back 42 pozy | 🟡 eksport i read-back lokalny OK; RobotStudio nieobecny, składnia niezweryfikowana kompilatorem vendor |
| Mapowanie zdarzeń na I/O | post.base | test_review_fixes | ✅ gotowe |
| Przybliżone kolizje kapsuła–prostopadłościan w próbkach trajektorii | collision.checker, program.simulator, pipeline | test_collision | 🟡 ostrzeżenia i API mapy par; GUI ma przełącznik kontroli w symulacji i eksporcie; bez edytora mapy, mesh/FCL, samokolizji i geometrii OEM |
| GUI (menu, drzewo, 3D, F6 przez walidator, raport i podgląd osiągalności projektu) | gui | test_gui_spec, test_render | 🟡 podgląd osiągalności po obliczeniu projektu; edycja gizmem i pozostały workflow otwarte |
| Selekcja elementów sceny z widoku 3D | gui.viewport, gui.render | test_gui_spec, test_render | ✅ kliknięcie pickowalnej geometrii synchronizuje zaznaczenie drzewa i panel właściwości |
| Edycja nazwy i pozy elementu sceny w GUI | gui.handlers, gui.main_window | test_gui_spec | ✅ panel/aktywacja dwuklikiem; jedna komenda Undo/Redo, walidacja pozy i unieważnianie wyniku CAM |
| Panel Projektu obróbki z raportem walidacji | gui.main_window, gui.handlers | test_gui_spec (w tym Qt offscreen) | ✅ tab dokowany obok właściwości; pokazuje pełne podsumowanie, lokalizacje NC, próbki, czas i ostrzeżenia; odświeżany po obliczeniu/eksporcie |
| Oś czasu symulacji GUI | gui.main_window, gui.handlers | test_gui_spec (Qt offscreen) | ✅ play/pauza, Stop przywracający początkowe osie, przewijanie próbek, regulacja tempa i aktualizacja widoku |
| Kalibracja TCP / układu / baz A↔B | coop.calibration | test_machining, test_gui_spec | 🟡 matematyka, CLI i przepływ GUI gotowe; brak interfejsu odczytu pozycji z robota |
| CLI kalibracji z JSON (TCP, bazy, układ 3-punktowy) | robocode calibrate | test_calibration_cli | ✅ walidacja danych, RMS, atomowy zapis wyniku |
| Dual-arm | coop.dual_arm | — | 🧪 eksperymentalne |
| Kolizje, sterowniki online, URDF | collision, drivers | — | ⏳ planowane |

---

## 12. Bezpieczeństwo przy robocie rzeczywistym

- Projekt domyślnie **offline**. Generowanie, wysyłanie i uruchamianie to osobne, jawne kroki.
- Przed pierwszym uruchomieniem programu z roboCODE: tryb T1 (zredukowana prędkość), krok po kroku,
  operator przy zezwoleniu, sprawdzenie TCP i bazy na sterowniku, przebieg „na sucho” nad detalem.
- Próba z 2026-10-15 nie została wykonana: brak sterownika OEM i dostępnej, skonfigurowanej celi;
  żaden ruch nie został wysłany. Program demo nie jest programem do uruchomienia na robocie.
- Sterownik online (v1.0, opcjonalnie): potwierdzenie operatora przed ruchem, identyfikacja robota
  i zgodność konfiguracji z manifestem, timeouty, obsługa utraty połączenia, **brak automatycznego
  wznawiania ruchu po błędzie**, stan bezpieczny przy niejednoznacznym stanie polecenia.
- Funkcje bezpieczeństwa maszyny realizuje i waliduje sterownik i stanowisko (analiza ryzyka, normy
  właściwe dla celi) — nie roboCODE i nie GUI.

---

## 13. Testy, CI i jakość

- Poziomy: jednostkowe (pozy, parser, FK/IK, walidator), integracyjne (NC → program → walidacja →
  post → manifest), bezpieczeństwa/regresji (błędne dane, blokada eksportu, nieaktualne pliki, fuzzing).
- Każdy znaleziony błąd = test reprodukujący przed poprawką.
- Testy właściwości bez dodatkowych zależności (losowe dane z ustalonym ziarnem).
- CI (`.github/workflows/ci.yml`): Python 3.10 i 3.12, `ruff --select F`, `pytest`, przebieg demo KRL i RAPID.
- Planowane: `mypy`, testy mutacyjne, skan zależności, golden files.

### 13.1 Kod szkicowany przez zewnętrzne boty
Mniej krytyczne moduły (np. `tools/`) mogą być szkicowane przez darmowe czaty AI. Zasady: bot dostaje
pełną specyfikację interfejsu i dialektu w promptcie; wynik **zawsze** przechodzi przegląd, poprawki
i testy (w tym przez pełny pipeline) przed włączeniem; docstring modułu podaje pochodzenie szkicu.
Kinematyka, parser, solver, walidator, post-procesory i pipeline nie są delegowane.

### 13.2 Przegląd zewnętrzny (ChatGPT, Gemini, Grok — 15.10.2026)
Każda uwaga oceniona i albo naprawiona z testem regresyjnym (`tests/test_review_fixes.py`), albo odrzucona
z uzasadnieniem.

| Źródło | Uwaga | Decyzja |
|---|---|---|
| Gemini | G1/G2/G3 bez wcześniejszego F tylko ostrzeżeniem | ✅ błąd w trybie ścisłym |
| Gemini | zagęszczanie orientacji wyłączone ≥ 179° | ✅ obrót Rodriguesa, oś prostopadła dla wektorów przeciwnych |
| Gemini | zdarzenia (wrzeciono) po najeździe | ❌ wrzeciono startuje w punkcie najazdu nad detalem — poprawnie |
| Gemini | samo `G1` bez współrzędnych = błąd | ❌ to poprawny G-code (zmiana trybu modalnego) |
| ChatGPT | MoveC symulowany jako 2 odcinki | ✅ prawdziwy łuk |
| ChatGPT | brak kontroli `joint_deg_s` | ✅ `JOINT_SPEED_RANGE` |
| ChatGPT | osie PTP/MoveAbsJ nieweryfikowane po poście | ✅ `POST_JOINT_*` |
| ChatGPT | zastępcze osie celu sprawdzały tylko pozycję | ✅ pozycja i orientacja |
| ChatGPT | niepoprawne tolerancje wyłączają kontrole | ✅ walidacja `SimTolerances` |
| ChatGPT | solver i sterownik mogą skończyć w innej konfiguracji | ✅ ostrzeżenie `CONFIG_DIFFERS` |
| ChatGPT | brak kolizji | ✅ jawne `COLLISION_NOT_CHECKED` w każdym raporcie; kolizje = v0.7 |
| ChatGPT | IK nie sprawdzane z FK | ❌ IK zwraca wynik tylko po osiągnięciu tolerancji 1e-3 mm / 1e-3° |
| ChatGPT | przekroczenie prędkości osi powinno blokować | ❌ sterownik zwalnia (bezpieczne); pozostaje ostrzeżeniem |
| Grok | `z{mm}` tworzy nieistniejące strefy (z2, z3) | ✅ inline `zonedata` |
| Grok (2. runda) | domyślna masa narzędzia 1 kg w RAPID | ✅ komentarz w pliku + ostrzeżenie `TOOL_LOAD_DEFAULT` (propozycję 0,001 kg odrzucono — zaniżanie masy jest gorsze) |
| Grok (2. runda) | `\WObj` przy MoveAbsJ | ✅ usunięte (było nieszkodliwe) |
| Grok (2. runda) | `ConfL\Off` | ❌ zostaje: symulator modeluje ciągłość osi tak jak sterownik z ConfL\Off |
| Grok | `robconf [0,0,0,0]` | ✅ z osi robota |
| Grok | brak `$APO.CPTP`/prędkości PTP, CIRC bez `$ORI_TYPE`/`$CIRC_TYPE` | ✅ nagłówek KRL, PTP dokładny |
| Grok | niezdefiniowane podprogramy | ✅ mapowanie zdarzeń; bez mapy — bezpieczna awaria ładowania + ostrzeżenie |
| Review | `BAS(#VEL_PTP,x)` dostawał °/s zamiast % | ✅ procent vmax |
| Grok | z1 nie istnieje w RAPID | ❌ z1 jest predefiniowane |
| Grok | `$ACT_TOOL`/`BAS(#TOOL,n)` | ❌ celowo nie nadpisujemy tablic TOOL_DATA/BASE_DATA sterownika |
| Grok | znak kwaternionu, `joint_sign` | ❌ już obsłużone (w>0; znaki osi w modelu kinematycznym) |

## 14. Definicja ukończenia funkcji

Funkcja jest gotowa, gdy: ma specyfikację i ograniczenia w tym dokumencie; ma testy; błędne dane
obsługuje jawnie; diagnostyka wskazuje przyczynę i miejsce; dokumentacja zgadza się z kodem; CI przechodzi;
funkcje ruchu rzeczywistego przeszły walidację w docelowym środowisku. Stan w §11 aktualizuje się w tym
samym commicie.

## 15. Jak uruchomić

```bash
pip install -e .[dev]            # GUI: pip install -e .[dev,gui]
pytest
python -m robocode.cli demo                                    # G-code demo → KRL + manifest
python -m robocode.cli machine examples/square.nc --post abb --out out/
python -m robocode.cli machine plik.nc --station cela.rcs.json --zrange 180 --zstep 20 --approach 50
python -m robocode.cli report out/Machining.manifest.json        # raport HTML obok manifestu
python -m robocode.cli samples examples/samples                  # 7 przykładowych programów NC
```
Windows: `build_exe.bat` buduje `roboCODE.exe` (launcher z menu, `robocode/launcher.py`, PyInstaller
`--onefile`, bez Qt); `roboCODE.bat` uruchamia launcher bez budowania. Wyniki w `out/` obok exe.

Kod wyjścia: `0` = plik wyeksportowany, `2` = eksport zablokowany (szczegóły w raporcie i manifeście).

---

# Część B — specyfikacja funkcji docelowych

Poniżej szczegółowa specyfikacja funkcji i GUI z pierwotnego README (zachowana bez zmian
merytorycznych). Kolejność realizacji wyznacza §10, a stan §11.

## B1. Funkcje (backend)

### B1.1 Stacja i drzewo elementów
- Stacja = drzewo elementów. Typy: `Station`, `Frame` (układ odniesienia), `Robot`, `Tool` (TCP),
  `Object` (geometria STL/STEP), `Target`, `Program`, `MachiningProject`, `Curve`, `Points`.
- Każdy element ma: `name`, `parent`, `pose` (względem rodzica), `visible`, `children`.
- Przeciągnięcie elementu na inny = zmiana rodzica z zachowaniem pozycji absolutnej (opcja: bez zachowania).
- Wiele stacji otwartych jednocześnie.
- Aktywny układ i aktywne narzędzie per robot (używane przy uczeniu celów i nowych programach).
- Operacje: dodaj, usuń, zmień nazwę (F2), pokaż/ukryj (F7), kopiuj/wytnij/wklej (z dziećmi), scal obiekty.

### B1.2 Pozy i orientacje
- Reprezentacje: XYZ + Euler w różnych kolejnościach (XYZ statyczne, ZY'X'' – KUKA, XY'Z'' itd.),
  XYZ + kwaternion (ABB), macierz 4x4.
- Każdy robot ma domyślną konwencję zgodną z kontrolerem; post-procesor sam przelicza.
- Funkcje: odwróć pozę, wyzeruj (jednostkowa), kopiuj/wklej jako tablica lub macierz.

### B1.3 Robot
- Model: kinematyka DH (6 osi) + opcjonalne osie zewnętrzne (szyna, obrotnica, pozycjoner).
- FK, IK (analityczne dla typowych 6R z nadgarstkiem sferycznym; numeryczne jako fallback).
- Lista wszystkich rozwiązań IK (do 8 konfiguracji dla 6R) z flagami F/R, U/D, F/N.
- Limity osi (sprzętowe + programowe), prędkości i przyspieszenia osi.
- Pozycja domowa, przestrzeń robocza (podgląd).
- Parametry kinematyczne edytowalne (np. po kalibracji).

### B1.4 Narzędzie (TCP)
- TCP względem kołnierza lub względem innego TCP (np. frez = wrzeciono + offset Z).
- Geometria narzędzia do wizualizacji i kolizji, skala, przesunięcie geometrii bez zmiany TCP.
- Kalibracja TCP z N konfiguracji przegubów dotykających jednego punktu (metoda najmniejszych kwadratów).
- GUI: „Narzędzia CNC → Kalibruj TCP…” wczytuje JSON `{"flange_poses":[...]}`, oblicza TCP i RMS,
  wymaga potwierdzenia zastosowania do zaznaczonego lub aktywnego narzędzia. Zmiana zachowuje jego
  orientację, jest dostępna przez Cofnij/Ponów i unieważnia wynik bieżącego projektu obróbki.
- CLI: `robocode calibrate tcp pomiary.json --out wynik.json`; wejście ma postać
  `{"flange_poses": [macierz_4x4, ...]}`. Wynik podaje `tcp_mm`, RMS w mm i liczbę próbek.

### B1.5 Układy odniesienia
- Zagnieżdżanie układów (stół -> detal).
- Kalibracja układu: metoda 3 punktów (początek, punkt na osi X, punkt w płaszczyźnie XY)
  oraz z N punktów (dopasowanie sztywne, algorytm Kabscha).
- GUI: „Narzędzia CNC → Kalibruj układ…” przyjmuje JSON z trzema punktami (`origin`, `x_point`,
  `xy_point`) w bazie aktywnego robota albo odpowiadające punkty `points_in_A` / `points_in_B`.
  Wynik jest przeliczany do transformacji względem rodzica zaznaczonego/aktywnego układu, pokazywany
  operatorowi (wraz z RMS dla dopasowania N-punktowego) i stosowany dopiero po potwierdzeniu.
  Zmiana wspiera Cofnij/Ponów; wczytanie pomiarów nie jest automatycznym pomiarem z robota.
- CLI: `robocode calibrate frame3 pomiary.json` przyjmuje pola `origin`, `x_point`, `xy_point`;
  `robocode calibrate bases pomiary.json` przyjmuje odpowiadające sobie listy `points_in_A`
  i `points_in_B` (co najmniej 3 niewspółliniowe punkty). Współrzędne są w mm, wynik zawiera
  macierz `T_base_frame` albo `T_A_B` (relacja `p_A = T_A_B @ p_B`) i RMS dla kalibracji baz.
- Wynik JSON jest zapisywany atomowo; niepoprawne lub zdegenerowane pomiary kończą komendę błędem.

### B1.6 Cele (targets)
- Cel kartezjański (zachowuje pozę względem układu) lub przegubowy (zachowuje wartości osi).
- „Naucz cel” (Ctrl+T) z bieżącej pozycji robota do aktywnego układu i narzędzia.
- „Cele na powierzchni”: klik na geometrii -> cel z normalną.
- Przesuń robota do celu, pokaż inne konfiguracje.

### B1.7 Programy i instrukcje
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

### B1.8 Symulacja
- Uruchom (dwuklik programu), pauza, stop, krok po instrukcji, „start od tej instrukcji”.
- Pasek osi czasu (przewijanie w przód i w tył).
- Współczynnik prędkości symulacji (domyślnie 5x, szybki tryb 100x).
- Ślad TCP (trace), czas cyklu, sprawdzanie kolizji w trakcie (obiekty w kolizji na czerwono).
- W GUI dostępne jest podstawowe odtwarzanie udanej symulacji: play/pauza, Stop i przewijanie
  próbek suwakiem. Wybór 1×/5×/20×/100× wyznacza odstęp timera z czasu cyklu i liczby próbek;
  ograniczenia timera mogą odbiegać od rzeczywistego czasu. Stop przywraca pozycję osi sprzed
  obliczenia; zmiana stacji lub unieważnienie projektu/programu usuwa nieaktualne próbki.
  Odtwarzanie jest podglądem offline, a nie czasem rzeczywistym sterownika.

### B1.9 Generowanie programu (post-procesory)
- Generuj (F6), generuj jako… (Shift+F6), wyślij do robota (Ctrl+F6, FTP), wybór post-procesora per robot.
- Opcje: dzielenie długich programów (max linii na plik), łuki -> odcinki liniowe (max długość łuku),
  minimalny krok, wstawianie podprogramów inline, folder wyjściowy.
- Post-procesor to klasa Pythona z metodami: `ProgStart`, `ProgFinish`, `MoveJ`, `MoveL`, `MoveC`,
  `setFrame`, `setTool`, `setSpeed`, `setZoneData`, `Pause`, `RunCode`, `RunMessage`, `setDO`, `waitDI`,
  `ProgSave`.

### B1.10 Projekt obróbki robotem (najważniejsze dla nas)
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

### B1.11 Tryb kooperatywny dwóch ramion (wyróżnik roboCODE)
- Robot A trzyma detal (remote TCP), robot B wrzeciono.
- Wejście: ścieżka 5-osiowa (pozycja + wektor narzędzia względem detalu).
- Solver dzieli ruch: A orientuje detal tak, by punkt obróbki był w strefie najwyższej sztywności
  i manipulowalności robota B; B wykonuje ruch względny.
- Kryteria kosztu: odległość od osobliwości (manipulowalność), limity osi, ciągłość przegubów,
  odległość od strefy preferowanej, kolizje A-B.
- Kalibracja baz A<->B z ≥3 wspólnych punktów (Kabsch).
- Eksport: dwa zsynchronizowane programy (znaczniki synchronizacji co N punktów) albo
  jeden program dla sterowania skoordynowanego producenta.

### B1.12 Narzędzia dodatkowe (kolejne wersje)
Pomiar punktów 3D, zmiana koloru, mapa kolizji, tworzenie/edycja krzywych z geometrii,
eksport symulacji do HTML 3D, kalibracja robota, strefy bezpieczeństwa, monitor I/O, pad do jogowania.

---

## B2. GUI

GUI definiujemy **deklaratywnie** w `robocode/gui/actions.py` (id, etykieta PL/EN, skrót, ikona,
komenda backendu). `main_window.py` tylko to renderuje. Dzięki temu testy sprawdzają kompletność
GUI bez uruchamiania Qt.

### B2.1 Układ okna
- **Menu główne** (góra), **pasek narzędzi** (pod menu, z opcją etykiet tekstowych),
  **drzewo stacji** (lewy panel, zmiana szerokości Ctrl+kółko), **widok 3D** (środek),
  **panel właściwości** (prawy, dokowalny), **pasek stanu** (dół, podpowiedzi),
  **oś czasu symulacji** (dokowany pasek na dole).
- Panel właściwości pokazuje nazwę i aktualną pozę względną elementu; przycisk „Edytuj właściwości…”
  lub dwuklik zaznaczonego elementu otwiera edytor XYZ [mm] i Rx/Ry/Rz [°]. Zastosowanie wymaga
  niepustej, unikatowej nazwy i poprawnej macierzy pozy. Całość jest jedną operacją Undo/Redo.
  Zmiana unieważnia poprzedni wynik projektu obróbki; przed dalszym użyciem ścieżki trzeba ją przeliczyć.
- Kliknięcie pickowalnej geometrii elementu w widoku 3D zaznacza go w drzewie i panelu właściwości.
  Zaznaczanie wspiera obecnie pojedynczy element; podłoga, ścieżki/overlay i elementy liniowe nie są
  wybierane. Zaznaczenie w drzewie działa również bez dostępnego renderera 3D.
- Dwuklik elementu drzewa otwiera edytor nazwy i pozy. Gizmo do przesuwania/obracania w widoku 3D
  nadal pozostaje do wdrożenia.
- Oś czasu symulacji udostępnia play/pauzę, Stop, suwak próbek, licznik bieżącej próbki i wybór
  prędkości 1×/5×/20×/100× (domyślnie 5×); akcja „Szybka symulacja” ustawia 100×, a wyłączenie
  wraca do 5×. Sterowanie jest aktywne po udanej symulacji;
  odtwarzanie aktualizuje podgląd 3D, a Stop przywraca pozycję początkową robota.
- Akcja „Sprawdzaj kolizje” jest domyślnie włączona i przełącza przybliżone kontrole proxy zarówno
  w symulacji GUI, jak i walidacji eksportu F6. Wyłączenie jej ustawia pustą listę par i nie wyłącza
  innych walidacji; brak edytora mapy oraz ograniczenia proxy opisano w §6.7.

### B2.2 Menu
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

### B2.3 Pasek narzędzi (domyślny)
Otwórz · Biblioteka · Zapisz · Cofnij · Ponów · Dodaj układ · Naucz cel · Dopasuj wszystko · Izometryczny ·
Przesuń układ (Alt) · Przesuń TCP (Alt+Shift) · Kolizje (przełącznik) · Szybka symulacja · Pauza ·
Dodaj program · Dodaj makro · MoveJ · MoveL · Eksport symulacji.

### B2.4 Nawigacja 3D
LPM: zaznacz (Ctrl: wielokrotne) · ŚPM: przesuń (Ctrl+Alt+LPM) · PPM: obrót (Ctrl+Shift+LPM) ·
kółko: zoom (Shift+LPM). Alt + przeciąganie: przesuwanie układu; Alt+Shift: przesuwanie TCP
(gizmo z osiami i pierścieniami obrotu). Menu kontekstowe w 3D z tymi samymi akcjami (laptop bez myszy).
Tryby nawigacji do wyboru w Opcjach (zgodne z popularnymi CAD).

### B2.5 Panele
- **Panel robota**: jog osi (suwaki + pola, kółko myszy zmienia wartość), limity (dwuklik = edycja),
  kopiuj/wklej osie, TCP względem kołnierza, układ względem bazy, TCP względem układu (edycja = ruch),
  inne konfiguracje, Parametry (DH, post-procesor).
- **Narzędzie**: poza TCP, rodzic (kołnierz/inne TCP), „Więcej opcji” (skala, przesunięcie geometrii).
- **Układ**: poza względem rodzica, wybór formatu orientacji, przyciski kopiuj/wklej/odwróć/zeruj.
- **Cel**: poza + osie, kartezjański/przegubowy, przesuń do celu, konfiguracje.
- **Projekt obróbki** (dokowalny): podsumowanie liczby osiągalnych próbek z numerami pierwszych
  próbek niedostępnych; punkty ścieżki są zielone/czerwone po obliczeniu projektu. Wybór robota/
  układu/narzędzia, wejście ścieżki, zdarzenia,
  najazd/odjazd (Dodaj/Usuń), path-to-tool offset z podglądem i kółkiem myszy, algorytm,
  optymalizacja (±kąt, krok), konfiguracja startowa, przyciski Aktualizuj/Symuluj, status ✓/✗.
- **Opcje**: Ogólne, Wyświetlanie, Ruch (prędkości symulacji), Program (dzielenie, łuki, inline,
  folder), CAM (domyślne zdarzenia, najazd), CAD (dokładność importu), Python.

### B2.6 Skróty (komplet)
F1 pomoc · F2 nazwa · F3 opcje celu · F6 generuj · Shift+F6 generuj jako · Ctrl+F6 wyślij ·
F7 pokaż/ukryj · Alt+0 dopasuj · Ctrl+1 ostatni plik · Ctrl+T naucz cel · Ctrl+Shift+T cele na powierzchni ·
Ctrl+M projekt obróbki · Spacja szybka symulacja · Backspace pauza · Esc stop / wyjście z trybu wyboru ·
`/` tekst · `+`/`−` rozmiar układów · `*` przestrzeń robocza.

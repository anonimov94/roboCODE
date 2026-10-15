# roboCODE

Otwarte narzędzie do **programowania robotów offline (OLP)** i **obróbki robotem**: z pliku G-code
powstaje sprawdzony program dla kontrolera KUKA (KRL) lub ABB (RAPID). Wyróżnik w dalszej
kolejności: frezowanie kooperatywne dwoma ramionami (jedno trzyma detal, drugie wrzeciono).

> **Wymagania, konwencje, architektura, roadmapa i stan prac są w jednym dokumencie:
> [`docs/SPECYFIKACJA.md`](docs/SPECYFIKACJA.md).** W razie rozbieżności obowiązuje ten plik.
> Co zrobione i co dalej (kolejka prac): [`docs/ROADMAP.txt`](docs/ROADMAP.txt).
> Bot programujący: przeczytaj go w całości przed pierwszą zmianą, zwłaszcza §2 (zasady), §3 (decyzje)
> i §4 (konwencje).

## Stan: proof of concept 0.2 (jeden robot, pełny przepływ)

```
plik NC ─► parser (stan modalny, tryb ścisły) ─► solver (obrót Z, najazd/odjazd)
        ─► walidacja całej trajektorii ─► KRL/RAPID w pamięci ─► odczyt zwrotny poz
        ─► zapis atomowy pliku + manifest.json (raport, wersje, sumy SHA-256)
```
Błąd krytyczny na którymkolwiek etapie = **brak pliku dla robota** i raport wskazujący linię NC.

## Launcher dla Windows (roboCODE.exe)

1. Dwuklik **`build_exe.bat`**: tworzy `.venv` (jeśli go nie ma), instaluje zależności i PyInstaller,
   buduje **`roboCODE.exe`** w folderze projektu (~30 MB, jeden plik, bez GUI Qt).
2. Dwuklik **`roboCODE.exe`** → menu: demo, obróbka własnego pliku NC (okno wyboru pliku), raport HTML,
   przykładowe pliki NC, folder wyników. Wyniki trafiają do folderu `out` obok exe.
3. `roboCODE.exe machine plik.nc --post abb` działa jak CLI. Bez budowania: dwuklik `roboCODE.bat`.

## Szybki start

```bash
pip install -e .[dev]          # z GUI: pip install -e .[dev,gui]
pytest
python -m robocode.cli demo                                         # demo → out/Machining.src
python -m robocode.cli machine examples/square.nc --post abb --out out/
python -m robocode.cli machine plik.nc --events examples/events_kuka.json   # wrzeciono jako $OUT/$ANOUT
python -m robocode.cli report out/Machining.manifest.json          # raport HTML
python -m robocode.cli samples examples/samples                    # przykładowe pliki NC
```

Przykładowy wynik:
```
✓ Zapisano Machining.src (ostrzeżenia: 3)
Walidacja: OK | błędy: 0 | ostrzeżenia: 3 | próbek trajektorii: 134 | czas cyklu ≈ 29.4 s
  ⚠ [parse/PARSER] linia 5: brak G20/G21 przed pierwszym ruchem – przyjęto mm (G21)
  ⚠ [static/CONTROLLER_SUBPROGRAMS] Program wywołuje podprogramy, które muszą istnieć na sterowniku: …
  ⚠ [trajectory/COLLISION_NOT_CHECKED] Kolizje (robot, narzędzie, detal, mocowania) NIE są jeszcze sprawdzane …
Manifest: out/Machining.manifest.json
```

GUI (zamrożone do v0.4, patrz specyfikacja §3 D1): `python -m robocode.cli gui`.

![Widok ogólny](docs/gui_demo.png)
![Ścieżka z bliska](docs/gui_path.png)

## Ważne ograniczenia

- Model robota w repo jest **syntetyczny** (`generic6r_900`), nie odpowiada żadnemu produktowi.
- Pliki KRL/RAPID **nie były jeszcze weryfikowane na prawdziwym sterowniku**.
- Brak sprawdzania kolizji (v0.7).
- roboCODE nie zastępuje funkcji bezpieczeństwa robota ani procedur stanowiska (specyfikacja §12).

## Źródła wymagań

Specyfikacja scala: pierwotną instrukcję projektu oraz wskazówki z `chatgpt.txt`, `gemini.txt`
i `roboCODE_roadmap_GROK.txt` (zachowane w repo jako materiał źródłowy).

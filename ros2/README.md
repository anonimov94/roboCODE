# roboCODE × ROS 2 / Gazebo

Ten folder to **przestrzeń robocza ROS 2** (colcon) z celą frezarską KUKA KR 10 R1100 sixx.
Zawiera cały projekt [ClaBirk/ROS2_Gazebo_Moveit_Simulation](https://github.com/ClaBirk/ROS2_Gazebo_Moveit_Simulation)
przeniesiony do roboCODE i **przeportowany z ROS 2 Humble + Gazebo Classic na ROS 2 Jazzy + Gazebo Harmonic**
(Gazebo Classic nie jest już wspierane), oraz most do roboCODE.

Podział ról:

```
Windows (PyCharm)                          Linux / WSL2 (ROS 2 Jazzy)
roboCODE: G-code → IK → walidacja          Gazebo Harmonic + ros2_control
  → KRL/RAPID + manifest                   MoveIt 2 (OMPL + Pilz) + RViz
  → <nazwa>.ros_trajectory.json  ───────►  trajectory_player → JointTrajectoryController
     (tylko po udanej walidacji)           krl_lin_milling   → program KRL (*.src) przez MoveIt/Pilz
```

roboCODE nadal działa samodzielnie na Windows (bez ROS). ROS 2 i Gazebo są warstwą symulacji
fizycznej i wykonania: odtwarzają dokładnie tę trajektorię osi, którą roboCODE sprawdził.

## Pakiety

| Pakiet | Co zawiera | Skąd (ClaBirk) |
|---|---|---|
| `robocode_kr10_description` | URDF/xacro celi (obudowa, drzwi, stół, płyta, KR10, uchwyt, wrzeciono), siatki STL, `ros2_control` (Gazebo/mock), kontrolery, podgląd RViz | `ros_sim` (urdf, meshes, config) |
| `robocode_kr10_moveit_config` | SRDF, kinematyka KDL, limity osi, limity Pilz, kontrolery MoveIt, RViz | `kr10zelle_moveit_config` |
| `robocode_bringup` | `sim.launch.py`: Gazebo albo mock + kontrolery + MoveIt + RViz (+ odtworzenie trajektorii roboCODE) | `ros_sim/launch/*` |
| `robocode_ros` | `trajectory_player` (trajektoria roboCODE → kontroler, ścieżka TCP w RViz), `joint_points_demo` | `ros_milling` (Python) |
| `robocode_milling_cpp` | `krl_lin_milling`: program KUKA KRL (`LIN`, `$VEL.CP`) → MoveIt/Pilz; przykładowe programy z Fusion 360 | `ros_milling_cpp` |

Zmiany względem oryginału (poza portem na Jazzy/Harmonic):
- jedna, spójna nazwa osi `joint_1..joint_6` (oryginał mieszał `joint1` i `joint_1` → kontrolery w Gazebo nie działały);
- poprawione bezwładności (ujemne i ~1000 kg·m² dla uchwytu), kolizja wrzeciona jako walec zamiast siatki 100 tys. trójkątów;
- `-1.57`/`1.57` zastąpione dokładnym `π/2`, dzięki czemu URDF zgadza się z modelem roboCODE co do 1e-6 mm;
- nowe ramki: `tool0` (kołnierz, Z na zewnątrz – jak w roboCODE) i `tcp` (czubek wrzeciona – **szacunek z siatki, zmierz rzeczywisty TCP**);
- grupa MoveIt `kr10_arm` kończy się na `tcp`, więc węzeł KRL nie musi ręcznie odejmować przesunięć `link_6 → narzędzie`;
- ścieżka pliku KRL i przesunięcia detalu są parametrami (oryginał miał wkompilowane `/home/claudius/...`);
- uproszczone demo węzłów Python (`joint_points_topic`) zastępuje `trajectory_player`; zdjęcia rysunków
  z `nc_program/` nie zostały skopiowane (są w oryginalnym repo), podobnie nieużywane warianty siatek.

Zgodność modelu sprawdzają testy roboCODE: `tests/test_kr10_model.py` (FK roboCODE = łańcuch URDF)
i `tests/test_ros_trajectory.py` (eksport trajektorii i zgodność formatu z `robocode_ros`).

## Instalacja (Windows → WSL2 z Ubuntu 24.04)

1. W PowerShell (administrator): `wsl --install -d Ubuntu-24.04`, uruchom ponownie komputer, otwórz „Ubuntu”.
   Windows 11 wyświetla okna Gazebo/RViz bez dodatkowej konfiguracji (WSLg).
2. Zainstaluj ROS 2 Jazzy według oficjalnej instrukcji (pakiety deb):
   <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html> (wariant `ros-jazzy-desktop`).
3. Pakiety używane przez celę:

```bash
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-moveit ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui \
  python3-colcon-common-extensions
```

4. Przestrzeń robocza (kod roboCODE leży na dysku Windows; budowanie na `/mnt/c` jest wolne,
   więc w WSL tworzymy tylko dowiązanie do folderu `ros2`):

```bash
mkdir -p ~/robocode_ws/src && cd ~/robocode_ws/src
ln -s /mnt/c/Users/pc/Documents/PythonProjects/roboCODE/ros2 robocode
cd ~/robocode_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash        # dopisz do ~/.bashrc razem z /opt/ros/jazzy/setup.bash
```

## Uruchomienie

```bash
ros2 launch robocode_bringup sim.launch.py                  # Gazebo + MoveIt + RViz
ros2 launch robocode_bringup sim.launch.py sim:=mock        # bez fizyki (szybko; gdy Gazebo w WSL działa słabo)
ros2 launch robocode_kr10_description view.launch.py        # sam model z suwakami osi
```

### Program z roboCODE w Gazebo

Na Windows (PyCharm / konsola w folderze roboCODE):

```bash
python -m robocode.cli machine examples/square.nc --station examples/kr10_cell.rcs.json --ros --out out
```

Powstaje `out/Machining.src`, `out/Machining.manifest.json` i **`out/Machining.ros_trajectory.json`**
(tylko gdy walidacja przeszła). W WSL:

```bash
ros2 launch robocode_bringup sim.launch.py moveit:=false \
  trajectory_file:=/mnt/c/Users/pc/Documents/PythonProjects/roboCODE/out/Machining.ros_trajectory.json
# albo do już uruchomionej symulacji:
ros2 run robocode_ros trajectory_player --ros-args -p trajectory_file:=<plik> -p time_scale:=2.0
```

Robot najpierw powoli dojeżdża do pierwszego punktu, potem wykonuje program; ścieżka TCP jest widoczna
w RViz (`/robocode/path_markers`). `time_scale:=2.0` = połowa prędkości. Czasy w pliku to estymata
roboCODE (profil trapezowy), nie czas sterownika KUKA.

### Program KRL przez MoveIt (jak w oryginale)

```bash
ros2 launch robocode_bringup sim.launch.py
ros2 run robocode_milling_cpp krl_lin_milling --ros-args \
  -p krl_file:=$(ros2 pkg prefix robocode_milling_cpp)/share/robocode_milling_cpp/nc_program/sPlanen2.src
```

Każdy `LIN` jest planowany osobno przez Pilz (jak w oryginale), więc duże programy (`sTasche1.src`,
121 tys. ruchów) trwają bardzo długo. Do takich programów lepsza jest ścieżka roboCODE → `trajectory_player`.

## Ograniczenia

- Pakiety ROS 2 nie zostały jeszcze zbudowane ani uruchomione w ROS 2 Jazzy przy przenoszeniu do roboCODE
  (środowisko deweloperskie nie miało ROS). Sprawdzone: rozwinięcie xacro w obu trybach, zgodność łańcucha
  URDF z roboCODE, nazwy w SRDF, parser KRL na wszystkich programach przykładowych, testy formatu trajektorii.
  Pierwsze `colcon build` i uruchomienie trzeba zrobić u siebie; błędy zgłaszać jako zadania w ROADMAP.
- Symulacja (Gazebo, MoveIt, roboCODE) nie zastępuje bezpieczeństwa stanowiska (docs/SPECYFIKACJA.md §12).
  `trajectory_player` jest przeznaczony do symulacji, nie do sterowania prawdziwym robotem.
- Wymiary, siatki i masy są nominalne. TCP wrzeciona to szacunek z siatki STL.

## Licencja

Zawartość `ros2/` pochodzi z projektu ClaBirk/ROS2_Gazebo_Moveit_Simulation i jest objęta licencją
**GNU GPL v3** (plik `LICENSE` w tym folderze); zmiany w `ros2/` są udostępniane na tej samej licencji.
Autor oryginału prosi o link do jego repozytorium przy ulepszeniach i o nieużywanie go w płatnych kursach.
roboCODE (Python) komunikuje się z tymi pakietami wyłącznie przez pliki i interfejsy ROS 2, nie importuje ich kodu.

# go2_autonomy — Autonome Kartierung und Navigation des Unitree Go2 (Isaac Lab + ROS 2)

Bachelorarbeit. Instanziierung des Unitree Go2 in NVIDIA Isaac Lab, autonome
2D-Kartierung eines unbekannten Raums mit SLAM Toolbox, Nav2 und Frontier-Based
Exploration, mit anschliessender Rueckkehr zur Startpose.

Der Autonomiezyklus ist geschlossen und in zwanzig Evaluierungslaeufen
vermessen, zehn je Testumgebung. Die in der Arbeit berichteten Ergebnisse
beruhen auf dem Tag `v2.0-eval-final`. Der Explorationsknoten wurde seither
strukturell ueberarbeitet und verhaelt sich nachweislich identisch.

---

## Architektur

Zwei Teilsysteme, gekoppelt ueber DDS und eine gemeinsame Simulationszeit.

| | Teilsystem 1 (Host) | Teilsystem 2 (Container `welt2_nav:jazzy`) |
|---|---|---|
| Inhalt | Isaac-Lab-Szene, Go2, trainierte Lauf-Policy | SLAM Toolbox, Nav2, Frontier-Knoten |
| Publiziert | `/scan` `/odom` `/tf` `/tf_static` `/clock` | `/map`, `map->odom`, `/cmd_vel` |
| Abonniert | `/cmd_vel` | `/scan` `/tf` `/clock` |

Teilsystem 1 laeuft nativ auf dem Host und ist nicht containerisierbar, da die
Stabilitaet der Blackwell-GPU an Treiber, X11 und GPU-Pinning des Hosts gebunden
ist. Teilsystem 2 laeuft im Container ohne GPU.

Die Odometrie ist Ground-Truth aus der Simulation. Das ist eine bewusste
Entscheidung, da der Forschungsbeitrag die Explorations-Pipeline ist und nicht
die Zustandsschaetzung. Realer Drift bleibt damit ausgeblendet.

> Die Image- und Pfadnamen `welt2_nav` und `~/welt2_slam` tragen weiterhin die
> historische Bezeichnung. Sie sind reine Bezeichner und werden nicht umbenannt,
> um bestehende Images und Mounts nicht zu brechen.

---

## Struktur

```
isaac/       Teilsystem 1: Szene, Startwrapper, Deployment-Policy
nav2_slam/   Teilsystem 2: Dockerfiles, DDS-Profil, SLAM- und Nav2-Parameter
frontier/    Frontier-Explorer (eigener rclpy-Knoten, objektorientiert)
tools/       Passive Messknoten der Evaluierung
```

**`isaac/`**

| Datei | Aufgabe |
|---|---|
| `play_go2_ros_scan.py` | Szene, Go2 mit Lauf-Policy, 2D-LiDAR, ROS-2-Bruecke. Enthaelt beide Testumgebungen, waehlbar ueber `--szene`. |
| `run_go2_scan.sh` | Startwrapper. Aktiviert die venv, setzt die Bibliothekspfade der Bruecke, ruft Isaac Lab auf. |
| `policy.pt` | Deployment-Policy, siehe unten. |
| `patch_saeulenraum.py`, `patch_unigang.py` | Historische Szenen-Generatoren. Durch `--szene` abgeloest, ohne Funktion im aktuellen Ablauf. |

**`tools/`** — alle drei Werkzeuge sind rein passiv, abonnieren nur und greifen in keinen Regelkreis ein.

| Datei | Aufgabe |
|---|---|
| `eval_probe.py` | Erfasst Roll- und Nickwinkel des Rumpfes, die Bewegung `map->odom`, Stillstaende und die beiden Wandwinkel, sichert die Karte aus `/map`. Schreibt `probe_ergebnisse.csv`. |
| `eval_probe2.py` | Zaehlt Planerausfaelle, Wiederherstellungsmanoever, Bereinigungen der Kostenkarte, verworfene Kartennachrichten und Eingriffe des Kollisionsmonitors, misst die Odometriedistanz und schneidet `/rosout` mit. Schreibt `eval_gesamt.csv` und `<name>_explorer.log`. |
| `cmd_odom_probe4.py` | Diagnose der Befehlskette ueber vier Ebenen vom Regler bis zur gemessenen Bewegung. Grundlage des Befundes zum Drehbaustein. |

---

## Testumgebungen

Die Umgebung wird ueber das Argument `--szene` gewaehlt. Beide Geometrien liegen
fest in `play_go2_ros_scan.py`, eine Dateiaenderung ist nicht noetig.

```bash
run_go2_scan.sh --szene ringflur       # 30 m Ringflur, Tuernischen, 41 Prims (Vorgabe)
run_go2_scan.sh --szene saeulenraum    # 20 x 20 m Halle, 48 Saeulen
```

Zur Kontrolle nennt die `[SZENE]`-Zeile beim Start Abmessungen und Objektzahl.

| Umgebung | Praefix der Laeufe | Kontrolle |
|---|---|---|
| Ringflur | `final_uni_1` … `final_uni_10` | `[SZENE] Uni-Ringflur 30.0 m \| … \| 41 Prims` |
| Saeulenraum | `final_srm_1` … `final_srm_10` | `[SZENE] Saeulenraum 20.0 x 20.0 m \| 48 Saeulen …` |

---

## Umgebung (verifiziert)

| Komponente | Version |
|---|---|
| OS | Ubuntu 24.04 LTS, Kernel 6.17.0-1023-oem, X11 |
| GPU | NVIDIA RTX 5090 Laptop (Blackwell, sm_120) |
| NVIDIA-Treiber | 580.95.05 (Open Kernel Module) |
| Isaac Sim / Isaac Lab | 5.1.0 / 2.3.0 |
| Python (Isaac-venv) | 3.11, torch 2.7.0+cu128 |
| ROS 2 | Jazzy (Container `ros:jazzy-ros-base`) |
| DDS | `rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=0`, UDPv4 erzwungen (SHM aus) |
| Docker | 29.4.2, Container CPU-only |

---

## Images bauen

Das Nav2-Image erbt additiv vom SLAM-Image, die Reihenfolge ist zwingend.

```bash
cd nav2_slam
docker build -f Dockerfile.slam -t welt2_slam:jazzy .
docker build -f Dockerfile.nav  -t welt2_nav:jazzy  .
```

---

## Starten

Die Reihenfolge ist zwingend: Isaac, SLAM, TF-Gate, Nav2, RViz, Explorer. Fuer
einen Evaluierungslauf kommen die beiden Messknoten hinzu, siehe den naechsten
Abschnitt.

**Terminal 1 — Isaac (Host)**

```bash
sudo systemctl stop ollama          # GPU-Speicher freigeben
xhost +local:root                   # Display fuer RViz im Container freigeben
~/go2_autonomy/isaac/run_go2_scan.sh --szene ringflur
```

Der Wrapper reicht alle Argumente an `play_go2_ros_scan.py` durch. Warten auf
`Bridge aktiv` und `[SCAN-DIAG]`, dann offen lassen.

| Argument | Vorgabe | Bedeutung |
|---|---|---|
| `--szene` | `ringflur` | Testumgebung, `ringflur` oder `saeulenraum` |
| `--laser_z` | `0.4` | Montagehoehe des LiDAR ueber `base_link` [m] |
| `--policy_path` | absoluter Pfad | auf fremden Maschinen setzen |

**Terminal 2 — Container und SLAM**

```bash
docker rm -f welt2 2>/dev/null
docker run --rm -it --name welt2 \
  --network host --ipc=host \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/welt2_slam/maps:/root/maps \
  -v ~/go2_autonomy:/root/repo:ro \
  welt2_nav:jazzy bash

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
  slam_params_file:=/root/repo/nav2_slam/mapper_params_go2.yaml
```

`/root/repo` ist nur lesend eingehaengt. Der Container laeuft als root und darf
nicht in den Git-Baum schreiben. Warten auf
`Registering sensor: [Custom Described Lidar]`, dann offen lassen.

**TF-Gate — Pflicht vor Nav2**

```bash
docker exec -it welt2 bash
ros2 run tf2_ros tf2_echo odom base_link
```

Erst wenn eine echte Transform kommt, mit Strg-C beenden und weiter. Startet Nav2
vorher, bricht der Bringup ab und der Explorer sperrt alle Ziele.

**Terminal 3 — Nav2**

```bash
docker exec -it welt2 bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true \
  params_file:=/root/repo/nav2_slam/nav2_go2.yaml
```

Warten auf `Managed nodes are active`. Die Zeile `Creating bond timer...` ist die
letzte eines gesunden Bringups und kein Fehler. Ohne `params_file` greift der
`base_footprint`-Default und `/cmd_vel` bleibt leer.

**Terminal 4 — RViz**

```bash
docker exec -it welt2 bash
ros2 run rviz2 rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz \
  --ros-args -p use_sim_time:=true
```

MarkerArray-Topic auf `/frontiers`, Kostenkarten ausschalten, Map-Display auf
`Durability = Transient Local`. Marker: gruen sind die Grenzzellen, orange die
Zentroide, rot der beste Kandidat, gold das aktive Ziel.

**Terminal 5 — Frontier-Explorer**

```bash
docker exec -it welt2 bash
python3 /root/repo/frontier/frontier_explorer.py --ros-args -p use_sim_time:=true
```

Der Roboter steht beim Start am Spawn, die Home-Pose wird beim ersten gueltigen
TF-Fix `map->base_link` gemerkt. Erwarteter Abschluss:

```
[3c] ===> EXPLORATION FERTIG: keine Frontiers mehr, Karte stabil ...
[3d] ===> HOME ERREICHT: zurueck an der Startpose. Autonomiezyklus komplett.
```

---

## Evaluierungslauf

`<name>` ist der Laufname, etwa `final_srm_1`. Die beiden Messknoten starten vor
dem Explorer, damit `/rosout` vollstaendig mitlaeuft.

```bash
# Terminal 6
docker exec -it welt2 bash
python3 /root/repo/tools/eval_probe2.py <name> --ros-args -p use_sim_time:=true

# Terminal 7
docker exec -it welt2 bash
python3 /root/repo/tools/eval_probe.py <name> --ros-args -p use_sim_time:=true
```

Meldet der Explorer `HOME ERREICHT`, beide Messknoten mit Strg-C beenden.
`eval_probe.py` sichert die Karte selbst aus `/map`, ein `map_saver` ist nicht
noetig. Die Diagnose der Befehlskette laeuft bei Bedarf ueber
`cmd_odom_probe4.py` in einem eigenen Terminal.

Ablage im Container unter `/root/maps/eval`, auf dem Host unter
`~/welt2_slam/maps/eval/`.

| Datei | Inhalt |
|---|---|
| `probe_ergebnisse.csv` | eine Zeile je Lauf, Rumpflage, Nachfuehrung, Stillstaende, Wandwinkel, Kartenmasse |
| `eval_gesamt.csv` | eine Zeile je Lauf, Effizienz, Fehler- und Sicherungszaehler |
| `<name>.pgm`, `<name>.pgm.yaml` | die erzeugte Karte |
| `<name>_explorer.log` | Mitschnitt von `/rosout` |

Karten und Messdateien sind Artefakte und liegen bewusst ausserhalb des Repos.

---

## Kernparameter (Stand der Messreihe)

**Nav2** (`nav2_slam/nav2_go2.yaml`)
- Footprint (Rechteck): `[[0.35, 0.155], [0.35, -0.155], [-0.35, -0.155], [-0.35, 0.155]]`
- `inflation_radius: 0.70`, `cost_scaling_factor: 3.0` (beide Kostenkarten)
- `xy_goal_tolerance: 0.45`, `yaw_goal_tolerance: 3.15`
- Global: `nav2_smac_planner::SmacPlanner2D`, `allow_unknown: true`, `max_planning_time: 2.0`
- `FollowPath`: `RotationShimController` um MPPI, `motion_model: DiffDrive`, `vx_max: 0.5`, `wz_max: 1.9`
- RotationShim: `angular_dist_threshold: 0.785`, `max_angular_accel: 8.0`, `rotate_to_heading_angular_vel: 0.6`
- Kollisionsmonitor hinter dem `velocity_smoother`

**SLAM** (`nav2_slam/mapper_params_go2.yaml`)
- `base_frame: base_link`, `resolution: 0.05`, `map_update_interval: 2.0`
- `minimum_travel_distance: 0.5`, `loop_search_maximum_distance: 10.0`

**Frontier** (`frontier/frontier_explorer.py`, alle per `-p` einstellbar)
- `min_cluster_size: 5`, `occupied_threshold: 65`, `max_occupied_neighbors: 0`, `connectivity: 8`
- `size_weight: 0.05`, `min_goal_distance: 0.5`, `blacklist_radius: 0.5`, `max_snap_candidates: 80`
- `reachability_inflation: 0.3` verdickt Hindernisse vor der Detektion, sodass in zu engen Luecken keine Grenzzellen entstehen
- `goal_clearance: 0.0`, die laufende Revalidierung prueft allein die Belegung der Zielzelle
- `completion_patience: 5`, `map_stability_tol: 50`, `goal_timeout: 100.0`

---

## Bekannte Limitationen

- **Drehbaustein und gelernte Policy.** Eine zeitsynchrone Aufzeichnung der
  Befehlskette zeigte, dass der RotationShim seinen Drehbefehl an der gemessenen
  Drehrate verankert und je Regeltakt um `max_angular_accel / controller_frequency`
  anhebt. Bei stehendem Roboter bleibt der Bezugspunkt bei null und der Befehl
  verharrt. Lag er unter der Ansprechschwelle der Policy, blieb der Roboter stehen.
  Die Anhebung auf `8.0` loest die Verriegelung, die Halbierung der Drehrate auf
  `0.6` beseitigt das entstehende Ueberschwingen.
- **Wandartefakte.** An entfernten Waenden entstehen durch Winkelluecken des
  Laserscanners scheinbare Grenzzellen. Die Laeufe bleiben vollstaendig, fahren
  aber Umwege.
- **Ground-Truth-Odometrie** blendet realen Drift aus.

---

## Nicht im Repo (bewusst)

- **RL-Trainings-Checkpoints** (`model_*.pt`), Zwischenstaende des Trainings.
- **Gespeicherte Karten** (`*.pgm` und zugehoerige `*.yaml`), generierte Artefakte.
- **Messdateien der Evaluierung** (`*.csv`, `*_explorer.log`) unter `~/welt2_slam/maps/eval/`.
- **USD- und ONNX-Binaerdaten.**

Die Deployment-Policy `isaac/policy.pt` liegt dagegen im Repo. Nachtrainieren ist
wegen CUDA-Nichtdeterminismus nicht bit-reproduzierbar, sie ist damit ein Eingang
der Evaluierung und kein ableitbares Artefakt.

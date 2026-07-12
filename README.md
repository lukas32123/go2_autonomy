# go2_autonomy — Autonome Kartierung & Navigation des Unitree Go2 (IsaacLab + ROS 2)

Bachelorarbeit: Instanziierung des Unitree Go2 in NVIDIA IsaacLab, autonome
2D-Kartierung eines unbekannten Raums mittels SLAM Toolbox, Nav2 und
Frontier-Based Exploration, mit anschliessender Rueckkehr zur Startpose.

**Stand: `v1.0-autonomiezyklus`** — der Autonomiezyklus ist end-to-end
geschlossen: Exploration -> robustes Abschlusskriterium (3c) -> Return-to-Home
(3d), abgesichert durch einen Ziel-Timeout gegen unerreichbare Ziele.

---

## Zwei-Welten-Architektur

| | Welt 1 (Host) | Welt 2 (Container `welt2_nav:jazzy`) |
|---|---|---|
| Inhalt | IsaacLab-Szene, Go2, trainierte Lauf-Policy | SLAM Toolbox, Nav2, Frontier-Knoten |
| Publiziert | `/scan` `/odom` `/tf` `/tf_static` `/clock` | `/map`, `map->odom`, `/cmd_vel` |
| Abonniert | `/cmd_vel` | `/scan` `/tf` `/clock` |

Welt 1 laeuft nativ auf dem Host (nicht containerisierbar: die Blackwell-
Stabilitaet ist an Treiber/X11/GPU-Pinning des Hosts gebunden). Kopplung
Welt 1 <-> Welt 2 ueber DDS + `/clock`.

**Odometrie ist Ground-Truth aus der Simulation** (bewusste Entscheidung:
der Forschungsbeitrag ist die Explorations-Pipeline, nicht die
Zustandsschaetzung; Limitation: realer Drift wird ausgeblendet).

---

## Struktur

```
isaac/       Welt 1: IsaacLab-Szene (Ring-Flur), Startwrapper, Patch-Skript, Policy
nav2_slam/   Welt 2: Dockerfiles, DDS-Profil, SLAM- und Nav2-Parameter
frontier/    Frontier-Explorer (eigener rclpy-Knoten) — Kern der Arbeit
tools/       check_map_shear.py (Kartenqualitaet), patch_rotshim.py
docs/        Architektur-Doku, Logbuch, Expose
```

---

## Umgebung (verifiziert)

| Komponente | Version |
|---|---|
| OS | Ubuntu 24.04 LTS, Kernel 6.17.0-1023-oem, X11 (Wayland deaktiviert) |
| GPU | NVIDIA RTX 5090 Laptop (Blackwell, sm_120), MUX auf "Discrete" |
| NVIDIA-Treiber | 580.95.05 (Open Kernel Module) |
| Isaac Sim / Isaac Lab | 5.1.0 / 2.3.0 |
| Python (Isaac-venv) | 3.11, torch 2.7.0+cu128 |
| ROS 2 | Jazzy (Isaac-interne Bridge; Container: `ros:jazzy-ros-base`) |
| DDS | `rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=0`, **UDPv4 erzwungen (SHM aus)** |
| Docker | 29.4.2, Container CPU-only (GPU bleibt vollstaendig bei Welt 1) |

---

## Images bauen

Das Nav2-Image erbt additiv vom SLAM-Image — Reihenfolge ist zwingend.

```bash
cd nav2_slam
docker build -f Dockerfile.slam -t welt2_slam:jazzy .   # Basis: SLAM Toolbox, RViz, UDP-Profil
docker build -f Dockerfile.nav  -t welt2_nav:jazzy  .   # + Nav2
```

---

## Starten (5 Terminals) — Reihenfolge ist zwingend

### Terminal 1 — Isaac (Welt 1), Host

```bash
sudo systemctl stop ollama          # GPU-Last freigeben (Isaac braucht den vollen VRAM)
xhost +local:root                   # RViz im Container darf aufs Display
~/go2_autonomy/isaac/run_go2_scan.sh
```

Der Wrapper aktiviert die venv selbst und findet die Bridge-Libs per Glob.
Warten auf `Bridge aktiv` + `[SCAN-DIAG]`. **Offen lassen.**

> Auf einer *fremden* Maschine zusaetzlich den Policy-Pfad uebergeben:
> `~/go2_autonomy/isaac/run_go2_scan.sh --policy ~/go2_autonomy/isaac/policy.pt`

### Terminal 2 — Container + SLAM

```bash
docker rm -f welt2 2>/dev/null
docker run --rm -it --name welt2 \
  --network host --ipc=host \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/welt2_slam/maps:/root/maps \
  -v ~/welt2_frontier:/root/frontier \
  -v ~/go2_autonomy:/root/repo:ro \
  welt2_nav:jazzy bash
```

`/root/repo` ist **read-only**: Der Container laeuft als `root` und darf nicht in
den Git-Baum schreiben. Live-Edits vom **Host** aus funktionieren unveraendert.

Im Container:

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
  slam_params_file:=/root/repo/nav2_slam/mapper_params_go2.yaml
```

Warten auf `Registering sensor: [Custom Described Lidar]`. Danach Stille = normal.
**Offen lassen.**

### TF-Gate — PFLICHT vor Nav2

```bash
docker exec -it welt2 bash
ros2 run tf2_ros tf2_echo odom base_link
```

Erst wenn eine **echte Transform** kommt (nicht `does not exist`): Strg-C, weiter.
Startet Nav2 vorher, kommt `Aborting bringup`, alle Ziele werden abgelehnt und der
Explorer blacklistet alles.

### Terminal 3 — Nav2

```bash
docker exec -it welt2 bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true \
  params_file:=/root/repo/nav2_slam/nav2_go2.yaml
```

Warten auf `Managed nodes are active`. **`Creating bond timer...` ist die letzte
Zeile eines gesunden Bringups — kein Haenger, nicht abbrechen.** Offen lassen.

Ohne `params_file` greift der `base_footprint`-Default und `/cmd_vel` bleibt leer.

### Terminal 4 — RViz

```bash
docker exec -it welt2 bash
ros2 run rviz2 rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz \
  --ros-args -p use_sim_time:=true
```

Einstellungen: MarkerArray-Topic auf `/frontiers`, **Costmaps ausschalten**,
Map-Display `Durability = Transient Local`.
Marker: gruen = Frontier-Zellen, orange = Zentroide, gold = aktives Ziel,
rot = bester Kandidat. (Cyan/Magenta waere die Costmap — nicht verwechseln.)

### Terminal 5 — Frontier-Explorer

```bash
docker exec -it welt2 bash
python3 /root/repo/frontier/frontier_explorer.py --ros-args -p use_sim_time:=true
```

Der Roboter sollte beim Start **am Spawn stehen** — die Home-Pose wird beim ersten
gueltigen TF-Fix `map->base_link` gemerkt.

Erwarteter Abschluss:
```
[3c] ===> EXPLORATION FERTIG: keine Frontiers mehr, Karte stabil ...
[3d] ===> HOME ERREICHT: zurueck an der Startpose. Autonomiezyklus komplett.
```

---

## Karte sichern & Qualitaet messen

```bash
# im Container
ros2 run nav2_map_server map_saver_cli -f /root/maps/<name> -t /map

# auf dem Host (reines Python, kein numpy)
python3 tools/check_map_shear.py ~/welt2_slam/maps/<name>.pgm
```

Karten sind **Artefakte** und landen bewusst ausserhalb des Repos
(`~/welt2_slam/maps`), nicht im Git-Baum.

---

## Fallback: die alte Startanleitung (eingefroren)

Die Arbeitsordner `~/ros_smoketest`, `~/welt2_slam/maps` und `~/welt2_frontier`
enthalten weiterhin den **bekannt-guten v1.0-Stand** und sind im `docker run`
weiterhin gemountet. Geht im Repo etwas schief, laesst sich damit unveraendert
starten:

| | Repo (Quelle) | Fallback (eingefroren) |
|---|---|---|
| Isaac | `~/go2_autonomy/isaac/run_go2_scan.sh` | `~/ros_smoketest/run_go2_scan.sh` |
| SLAM-Params | `/root/repo/nav2_slam/mapper_params_go2.yaml` | `/root/maps/mapper_params_go2.yaml` |
| Nav2-Params | `/root/repo/nav2_slam/nav2_go2.yaml` | `/root/maps/nav2_go2.yaml` |
| Frontier | `/root/repo/frontier/frontier_explorer.py` | `/root/frontier/frontier_explorer.py` |

> **Regel 1 — Editiert wird ausschliesslich im Repo.** Der Fallback wird nicht
> gepflegt. Ein Sicherheitsnetz muss nicht aktuell sein, es muss bekannt-gut sein.
>
> **Regel 2 — NIE beide Wege gleichzeitig starten.** Beide sprechen
> `ROS_DOMAIN_ID=0`: zwei Isaac-Instanzen wuerden konkurrierend `/scan` und `/tf`
> publizieren, zwei Frontier-Knoten gleichzeitig Ziele senden.

---

## Betriebswissen (harte Regeln)

- **Startreihenfolge:** Isaac -> SLAM -> **TF-Gate** -> Nav2 -> RViz -> Frontier.
- **Nach jedem Config-Edit per `grep` verifizieren**, dass die Aenderung wirklich in
  der gelesenen Datei steht, bevor der Knoten neu startet. (Ist real schon einmal
  schiefgegangen.)
- **Keinen manuellen `twist_publisher` parallel** laufen lassen — Konkurrenz um `/cmd_vel`.
- Paste-Artefakte wie `use_sim_time:=trueue` sind harmlose Scheinfehler (ROS ignoriert
  unbekannte Args), aber sauber `:=true` tippen.

---

## Nicht im Repo (bewusst)

- **RL-Trainings-Checkpoints** (`model_*.pt`) — nur Zwischenstaende des Trainings.
  Der Trainingslauf liegt unter
  `~/IsaacLab/logs/rsl_rl/unitree_go2_flat/2026-06-18_11-25-44/`.
- **Gespeicherte Karten** (`*.pgm` / zugehoerige `*.yaml`) — generierte Artefakte.
- **USD-/ONNX-Binaerdaten.**

Die **Deployment-Policy** `isaac/policy.pt` (171 KB) liegt dagegen **im Repo**:
Nachtrainieren ist wegen CUDA-Nichtdeterminismus nicht bit-reproduzierbar, sie ist
damit ein *Eingang* der Evaluierung, kein ableitbares Artefakt.

---

## Kernparameter (Stand v1.0)

**Nav2** (`nav2_slam/nav2_go2.yaml`)
- Footprint (Rechteck, Go2-Steh-Envelope): `[[0.35, 0.155], [0.35, -0.155], [-0.35, -0.155], [-0.35, 0.155]]`
- `consider_footprint: true`, `inflation_radius: 0.70`
- `xy_goal_tolerance: 0.45`, `yaw_goal_tolerance: 3.15` (Endausrichtung fuer Exploration irrelevant)
- `FollowPath`: RotationShimController um MPPI (gegen Gross-Drehungs-Haenger)
- Global: NavFn, `allow_unknown: true`

**SLAM** (`nav2_slam/mapper_params_go2.yaml`)
- `base_frame: base_link` (upstream-Default waere `base_footprint`)
- `map_update_interval: 2.0`, `use_scan_matching: true`

**Frontier** (`frontier/frontier_explorer.py`, alle per `-p` tunebar)
- `min_cluster_size: 5`, `occupied_threshold: 65`, `max_occupied_neighbors: 0`
- `size_weight: 0.05` (Kosten = Distanz − size_weight·Clustergroesse)
- `min_goal_distance: 0.3`, `blacklist_radius: 0.5`
- `completion_patience: 5`, `map_stability_tol: 50` (3c: Abschluss nur auf stabiler Karte)
- `goal_timeout: 100.0` (Sicherheitsnetz gegen unerreichbare Ziele)
- `MAX_HOME_ATTEMPTS: 3` (Konstante)

---

## Bekannte Limitationen

- **Dreh-Ineffizienz (MPPI <-> gelernte Policy):** anfaengliches Hin-und-Her-Drehen,
  Ecken-Stalls. Der RotationShim entschaerft es, beseitigt es aber nicht.
- **Wand-Artefakt-Frontiers:** an entfernten Waenden entstehen durch LiDAR-Winkelluecken
  scheinbare Frontiers; Laeufe bleiben vollstaendig, aber mit Umwegen.
- **Ground-Truth-Odometrie** blendet realen Drift aus.

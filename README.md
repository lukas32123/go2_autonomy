
# go2_autonomy — Autonome Kartierung & Navigation des Unitree Go2 (Isaac Lab + ROS 2)

Bachelorarbeit: Instanziierung des Unitree Go2 in NVIDIA Isaac Lab, autonome
2D-Kartierung eines unbekannten Raums mittels SLAM Toolbox, Nav2 und
Frontier-Based Exploration, mit anschliessender Rueckkehr zur Startpose.

**Stand: `messreihe-2026-08`** — der Autonomiezyklus ist geschlossen
(Exploration, Abschlusskriterium, Rueckkehr zum Ausgangspunkt) und in zwanzig
Evaluierungslaeufen vermessen, zehn je Testumgebung. Dieser Tag ist der Stand,
auf dem alle in der Arbeit berichteten Ergebnisse beruhen.

---

## Architektur: zwei Teilsysteme

| | Teilsystem 1 (Host) | Teilsystem 2 (Container `welt2_nav:jazzy`) |
|---|---|---|
| Inhalt | Isaac-Lab-Szene, Go2, trainierte Lauf-Policy | SLAM Toolbox, Nav2, Frontier-Knoten |
| Publiziert | `/scan` `/odom` `/tf` `/tf_static` `/clock` | `/map`, `map->odom`, `/cmd_vel` |
| Abonniert | `/cmd_vel` | `/scan` `/tf` `/clock` |

Teilsystem 1 laeuft nativ auf dem Host (nicht containerisierbar: die Blackwell-
Stabilitaet ist an Treiber/X11/GPU-Pinning des Hosts gebunden). Kopplung ueber
DDS + `/clock`.

> Die Image- und Pfadnamen (`welt2_nav`, `~/welt2_slam`) tragen weiterhin die
> historische Bezeichnung. Sie sind Bezeichner und werden nicht umbenannt, um
> bestehende Mounts und Images nicht zu brechen.

**Odometrie ist Ground-Truth aus der Simulation** (bewusste Entscheidung:
der Forschungsbeitrag ist die Explorations-Pipeline, nicht die
Zustandsschaetzung; Limitation: realer Drift wird ausgeblendet).

---

## Struktur

```
isaac/       Teilsystem 1: Szene, Startwrapper, Szenen-Patches, Deployment-Policy
nav2_slam/   Teilsystem 2: Dockerfiles, DDS-Profil, SLAM- und Nav2-Parameter
frontier/    Frontier-Explorer (eigener rclpy-Knoten) — Kern der Arbeit
tools/       Messknoten fuer die Evaluierung + abgeloeste Vorlaeufer
```

### `isaac/`

| Datei | Aufgabe |
|---|---|
| `play_go2_ros_scan.py` | Szene, Go2 mit Lauf-Policy, 2D-LiDAR, ROS-2-Bruecke |
| `run_go2_scan.sh` | Startwrapper (venv, Bridge-Libs, Isaac-Lab-Aufruf) |
| `patch_saeulenraum.py` | schreibt `design_scene()` auf den **Saeulenraum** um |
| `patch_unigang.py` | schreibt `design_scene()` auf den **Ringflur** um |
| `patch_ringflur.py` | fruehere, kleinere Ringvariante (10 m) — nicht in der Messreihe verwendet |
| `policy.pt` | Deployment-Policy (siehe unten) |

### `tools/`

| Datei | Aufgabe |
|---|---|
| `eval_probe.py` | **In der Messreihe verwendet.** Passiver Messknoten: Roll/Nick des Rumpfes, Bewegung `map->odom`, Haenger-Erkennung, sichert die Karte aus `/map` und misst die Wandwinkel. Schreibt `probe_ergebnisse.csv`. |
| `eval_probe2.py` | **In der Messreihe verwendet.** Passiver Messknoten: Planerausfaelle, Wiederherstellungsmanoever, Costmap-Leerungen, verworfene Kartennachrichten, Kollisionsmonitor, `/odom`-Distanz, schneidet `/rosout` mit. Schreibt `eval_gesamt.csv` und `<name>_explorer.log`. |
| `cmd_odom_probe4.py` | **In der Messreihe verwendet.** Vier-Ebenen-Diagnose der Befehlskette vom Regler bis zur Ist-Bewegung. Grundlage des Befundes zum RotationShim. |
| `tilt_tf_probe.py`, `cmd_odom_probe.py`, `check_map_shear.py` | Vorlaeufer, in `eval_probe.py` zusammengefasst. Bleiben als Nachweis der Herkunft im Repo. |
| `shear_probe.py`, `patch_rotshim.py` | abgeloest, nicht mehr im Ablauf. |

---

## Testumgebungen

Beide Umgebungen entstehen dadurch, dass ein Patch-Skript `design_scene()` in
`play_go2_ros_scan.py` **ersetzt**. Es ist also immer nur eine Umgebung aktiv.
Beim ersten Lauf legt das Skript eine Sicherung an; ein zweiter Aufruf
ueberschreibt sie nicht.

```bash
python3 isaac/patch_saeulenraum.py    # 20 x 20 m Halle, 48 Saeulen
python3 isaac/patch_unigang.py        # 30 m Ringflur, Tuernischen, 41 Prims
```

Kontrolle beim Start von Isaac: die `[SZENE]`-Zeile nennt Abmessungen und
Anzahl der erzeugten Objekte.

| Umgebung | Praefix der Laeufe | Kontrolle |
|---|---|---|
| Saeulenraum | `final_srm_1` … `final_srm_10` | `[SZENE] Saeulenraum 20.0 x 20.0 m \| 48 Saeulen …` |
| Ringflur | `final_uni_1` … `final_uni_10` | `[SZENE] Uni-Ringflur 30.0 m \| … \| 41 Prims` |

> **Bekannte Einschraenkung.** Wer das Repo klont, bekommt genau die Umgebung,
> die zuletzt hineingeschrieben wurde. Die Umschaltung ueber ein Argument
> `--szene` ist vorgesehen, aber noch nicht umgesetzt.

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
| Docker | 29.4.2, Container CPU-only (GPU bleibt vollstaendig bei Teilsystem 1) |

---

## Images bauen

Das Nav2-Image erbt additiv vom SLAM-Image — Reihenfolge ist zwingend.

```bash
cd nav2_slam
docker build -f Dockerfile.slam -t welt2_slam:jazzy .   # Basis: SLAM Toolbox, RViz, UDP-Profil
docker build -f Dockerfile.nav  -t welt2_nav:jazzy  .   # + Nav2
```

---

## Starten — Reihenfolge ist zwingend

Fuer einen normalen Lauf reichen fuenf Terminals. Fuer einen **Evaluierungslauf**
kommen die beiden Messknoten hinzu, siehe den uebernaechsten Abschnitt.

### Terminal 1 — Isaac (Teilsystem 1), Host

```bash
sudo systemctl stop ollama          # GPU-Last freigeben (Isaac braucht den vollen VRAM)
xhost +local:root                   # RViz im Container darf aufs Display
~/go2_autonomy/isaac/run_go2_scan.sh
```

Der Wrapper aktiviert die venv selbst und findet die Bridge-Libs per Glob. Alle
weiteren Argumente reicht er an `play_go2_ros_scan.py` durch.

| Argument | Vorgabe | Bedeutung |
|---|---|---|
| `--laser_z` | `0.4` | Montagehoehe des LiDAR ueber `base_link` [m] |
| `--lidar_config` | `Slamtec_RPLIDAR_S2E` | 2D-LiDAR-Profil |
| `--cmd_timeout` | `0.5` | Watchdog [s]: ohne Twist -> Kommando 0 |
| `--policy_path` | absoluter Pfad | auf fremden Maschinen setzen |

Warten auf `Bridge aktiv` + `[SCAN-DIAG]`. **Offen lassen.**

> Auf einer *fremden* Maschine zusaetzlich den Policy-Pfad uebergeben:
> `~/go2_autonomy/isaac/run_go2_scan.sh --policy_path ~/go2_autonomy/isaac/policy.pt`

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

## Evaluierungslauf

Beide Messknoten sind **rein passiv**: sie abonnieren nur, publizieren nichts und
greifen in keinen Regelkreis ein. Ein Lauf kann durch sie nicht anders ausgehen
als ohne sie. `<name>` ist der Laufname, z. B. `final_srm_1`.

Reihenfolge: Isaac -> SLAM -> TF-Gate -> Nav2 -> RViz -> **eval_probe2** ->
**eval_probe** -> Explorer.

```bash
# Terminal 6 — vor dem Explorer starten, damit /rosout vollstaendig mitlaeuft
docker exec -it welt2 bash
python3 /root/repo/tools/eval_probe2.py <name> --ros-args -p use_sim_time:=true

# Terminal 7 — parallel
docker exec -it welt2 bash
python3 /root/repo/tools/eval_probe.py <name> --ros-args -p use_sim_time:=true
```

Meldet der Explorer `HOME ERREICHT`, beide Messknoten mit Strg-C beenden.
`eval_probe.py` sichert dabei die Karte selbst aus `/map`, ein `map_saver` ist
nicht noetig.

Ablage im Container unter `/root/maps/eval`, auf dem Host also
`~/welt2_slam/maps/eval/`:

| Datei | Inhalt |
|---|---|
| `probe_ergebnisse.csv` | eine Zeile je Lauf, Rumpflage, Nachfuehrung, Haenger, Wandwinkel, Kartenmasse |
| `eval_gesamt.csv` | eine Zeile je Lauf, Effizienz, Fehler- und Sicherungszaehler |
| `<name>.pgm` / `<name>.pgm.yaml` | die erzeugte Karte |
| `<name>_explorer.log` | Mitschnitt von `/rosout` |

Karten und Messdateien sind **Artefakte** und liegen bewusst ausserhalb des
Repos, nicht im Git-Baum.

Fuer die Diagnose der Befehlskette laeuft `cmd_odom_probe4.py` in einem eigenen
Terminal, unabhaengig von den beiden Messknoten:

```bash
python3 /root/repo/tools/cmd_odom_probe4.py --ros-args -p use_sim_time:=true
```

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
- **Vor einer Messreihe die Konfiguration einfrieren** und einen Git-Tag setzen.
  Die berichteten Ergebnisse sind nur dann einer Codeversion zuzuordnen.

---

## Nicht im Repo (bewusst)

- **RL-Trainings-Checkpoints** (`model_*.pt`) — nur Zwischenstaende des Trainings.
  Der Trainingslauf liegt unter
  `~/IsaacLab/logs/rsl_rl/unitree_go2_flat/2026-06-18_11-25-44/`.
- **Gespeicherte Karten** (`*.pgm` / zugehoerige `*.yaml`) — generierte Artefakte.
- **Messdateien der Evaluierung** (`*.csv`, `*_explorer.log`) — liegen unter
  `~/welt2_slam/maps/eval/`.
- **USD-/ONNX-Binaerdaten.**

Die **Deployment-Policy** `isaac/policy.pt` (171 KB) liegt dagegen **im Repo**:
Nachtrainieren ist wegen CUDA-Nichtdeterminismus nicht bit-reproduzierbar, sie ist
damit ein *Eingang* der Evaluierung, kein ableitbares Artefakt.

---

## Kernparameter (Stand der Messreihe)

**Nav2** (`nav2_slam/nav2_go2.yaml`)
- Footprint (Rechteck, Go2-Steh-Envelope): `[[0.35, 0.155], [0.35, -0.155], [-0.35, -0.155], [-0.35, 0.155]]`
- `inflation_radius: 0.70`, `cost_scaling_factor: 3.0` (beide Costmaps)
- `xy_goal_tolerance: 0.45`, `yaw_goal_tolerance: 3.15` (Endausrichtung fuer Exploration irrelevant)
- Global: **`nav2_smac_planner::SmacPlanner2D`**, `allow_unknown: true`,
  `max_planning_time: 2.0`, `cost_travel_multiplier: 2.0`
- `FollowPath`: `RotationShimController` um MPPI, `motion_model: DiffDrive`,
  `vx_max: 0.5`, `wz_max: 1.9`
- RotationShim: `angular_dist_threshold: 0.785`, `max_angular_accel: 8.0`,
  `rotate_to_heading_angular_vel: 0.6` (beide Werte gemessen hergeleitet, siehe unten)
- `CostCritic`: `consider_footprint: true`
- Kollisionsmonitor hinter dem `velocity_smoother`: `cmd_vel_smoothed -> cmd_vel`

**SLAM** (`nav2_slam/mapper_params_go2.yaml`)
- `base_frame: base_link` (upstream-Default waere `base_footprint`)
- `resolution: 0.05`, `map_update_interval: 2.0`, `use_scan_matching: true`
- `minimum_travel_distance: 0.5`, `loop_search_maximum_distance: 10.0`

**Frontier** (`frontier/frontier_explorer.py`, alle per `-p` tunebar)
- `min_cluster_size: 5`, `occupied_threshold: 65`, `max_occupied_neighbors: 0`,
  `connectivity: 8`
- `size_weight: 0.05` (Kosten = Distanz − size_weight·Clustergroesse)
- `min_goal_distance: 0.5`, `blacklist_radius: 0.5`, `max_snap_candidates: 80`
- `reachability_inflation: 0.3` — Hindernisse werden **vor** der Detektion
  verdickt, damit in zu engen Luecken gar keine Frontiers entstehen
- `goal_clearance: 0.0` — die geometrische Zielpruefung ist damit inaktiv; die
  laufende Revalidierung prueft allein die Belegung der Zielzelle
- `completion_patience: 5`, `map_stability_tol: 50` (Abschluss nur auf stabiler Karte)
- `goal_timeout: 100.0` (Sicherheitsnetz gegen unerreichbare Ziele)
- `MAX_HOME_ATTEMPTS: 3` (Konstante)

---

## Bekannte Limitationen

- **RotationShim und gelernte Policy.** Eine zeitsynchrone Aufzeichnung der
  Befehlskette (`cmd_odom_probe4.py`) zeigte, dass der Shim seinen Drehbefehl an
  der *gemessenen* Drehrate verankert und je Regeltakt um
  `max_angular_accel / controller_frequency` anhebt. Bei stehendem Roboter bleibt
  der Bezugspunkt bei null, der Befehl verharrt bei diesem Betrag. Lag er unter
  der Ansprechschwelle der Policy, blieb der Roboter stehen. Die Anhebung auf
  `8.0` loest die Verriegelung, die Halbierung der Drehrate auf `0.6` beseitigt
  das dadurch entstandene Ueberschwingen.
- **Wand-Artefakt-Frontiers:** an entfernten Waenden entstehen durch LiDAR-Winkelluecken
  scheinbare Frontiers; Laeufe bleiben vollstaendig, aber mit Umwegen.
- **Ground-Truth-Odometrie** blendet realen Drift aus.
- **Umgebungswechsel nur per Patch-Skript**, siehe oben.

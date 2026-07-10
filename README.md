# go2_autonomy — Autonome Kartierung Unitree Go2 (IsaacLab + ROS 2)

Stand: v1.0-autonomiezyklus. Autonomiezyklus geschlossen
(Exploration -> robuster Abschluss 3c -> Return-to-Home 3d, mit Ziel-Timeout).

## Struktur
- isaac/      Welt-1-Szene (IsaacLab) + Startskript
- nav2_slam/  Welt-2-Container: Dockerfile, DDS-Config, SLAM- & Nav2-Parameter
- frontier/   Frontier-Explorer (eigener rclpy-Knoten, Kern der Arbeit)
- tools/      Hilfsskripte (Kartenqualitaet, Patches)
- docs/       Architektur-Doku, Logbuch, Expose

## Nicht im Repo (bewusst)
- Trainierte Policy (Deployment):
  /home/kilab/IsaacLab/logs/rsl_rl/unitree_go2_flat/2026-06-18_11-25-44/exported/policy.pt
- RL-Trainings-Checkpoints (model_*.pt) und gespeicherte Karten (*.pgm)

## Umgebung (zu ergaenzen)
- ROS 2: [UNKLAR], IsaacSim/IsaacLab: [UNKLAR], NVIDIA-Treiber: [UNKLAR]

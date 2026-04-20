# FloriBot 4.0 - ROS 2 Jazzy Base

Dieses Repository enthält die Basis-Software für den **FloriBot 4.0**, in **ROS 2 Jazzy Jalisco**. Es bildet das Fundament der Robotersteuerung, verwaltet die Hardware-Kommunikation und stellt die Schnittstellen für Navigation und Teleoperation bereit.

## Projektbeschreibung

Die `FloriBot4.0_ROS2_Jazzy_Base` dient als Bindeglied zwischen der ROS 2-Umgebung und der physischen Hardware. Das Paket übernimmt die Berechnung der Kinematik, die Aufbereitung der Odometrie-Daten und die Kommunikation mit der speicherprogrammierbaren Steuerung (PLC/SPS).

## Architektur & Module

Das System ist in zwei Hauptknoten unterteilt, um eine saubere Trennung zwischen Logik und Hardware-Abstraktion zu gewährleisten:

* **`plc_connection`**: Dieser Knoten ist der direkte Draht zur Hardware. Er kapselt das Kommunikationsprotokoll zur PLC. Er sendet Soll-Geschwindigkeiten an die Motoren und empfängt Ist-Daten (Encoder-Werte) für die Rückkopplung.
* **`base_node`**: Die Intelligenz der Basis. Dieser Knoten abonniert `cmd_vel` (Twist-Nachrichten), wandelt diese mittels inverser Kinematik in Radgeschwindigkeiten um und gibt sie an den `plc_connection` Node weiter. Gleichzeitig berechnet er die Odometrie-Transformation ($TF$) und publiziert den Status des Roboters.

---

## Installation & Setup

### 1. Repository klonen

```bash
git clone https://github.com/Team-FloriBot/FloriBot4.0_ROS2_Jazzy_Base.git base_ws

```

### 2. Abhängigkeiten auflösen
```bash
cd ~/base_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

```

### 3. Projekt bauen
```bash
colcon build --symlink-install

```

### 4. Workspace sourcen
```bash
source ~/base_ws/install/setup.bash

```

## Nodes starten

```bash
ros2 launch base base_launch.py
```
in neuem Terminal:

```bash
ros2 launch plc_connection plc_connection_launch.py
```
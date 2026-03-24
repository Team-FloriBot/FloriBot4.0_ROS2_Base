# FloriBot4.0_ROS2_Base
Dieses Repository enthält die ROS2-Jazzy-Implementierung der mobilen Basis des FloriBot 4.0. Die Struktur orientiert sich bewusst an der klaren und modularen Organisation des FloriBot_Pi-Projekts, wurde jedoch vollständig neu und systematisch für ROS2 umgesetzt.

# Architektur

Die Base ist in zwei zentrale Komponenten aufgeteilt:

hardware_node
Schnittstelle zur realen Hardware über sensor_msgs/JointState.
Extrahiert:

Artikulationswinkel (Knickgelenk)
Radzustände (Position/Geschwindigkeit)

Publiziert:

/base/articulation_angle
/base/wheel_ticks4

kinematics_node
Implementiert die artikulierte Kinematik der Plattform.
Verarbeitet:

cmd_vel
Gelenkwinkel
Radzustände

Berechnet:

Soll-Radgeschwindigkeiten (/base/wheel_commands4)
Odometrie (/odom)
TF (odom -> base_link)
Herkunft der Kinematik

Die mathematische Modellierung basiert auf der bestehenden ROS1-Implementierung aus dem Advanced_Navigation-Repository. Die Softwarearchitektur wurde jedoch vollständig neu strukturiert und an ROS2 angepasst.

Designprinzipien
Klare Trennung von Hardware, Kinematik und Kommunikation
Verwendung standardisierter ROS2-Interfaces (cmd_vel, Odometry, tf)
Keine Abhängigkeit der Kinematik von TF als Eingangsgröße
Erweiterbarkeit für Navigation (Nav2) und Sensorfusion
Ziel

Dieses Paket stellt eine saubere, modulare und erweiterbare Grundlage für die weitere Entwicklung des FloriBot 4.0 in ROS2 dar.

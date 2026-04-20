# FloriBot4.0_ROS2_Base
Dieses Repository enthält die ROS2-Jazzy-Implementierung der mobilen Basis des FloriBot 4.0. 



# Herkunft der Kinematik

Die mathematische Modellierung basiert auf der bestehenden ROS1-Implementierung aus dem Advanced_Navigation-Repository. Die Softwarearchitektur wurde jedoch vollständig neu strukturiert und an ROS2 angepasst.

# Designprinzipien
Klare Trennung von Hardware, Kinematik und Kommunikation
Verwendung standardisierter ROS2-Interfaces (cmd_vel, Odometry, tf)
Keine Abhängigkeit der Kinematik von TF als Eingangsgröße
Erweiterbarkeit für Navigation (Nav2) und Sensorfusion
Ziel

Dieses Paket stellt eine modulare und erweiterbare Grundlage für die weitere Entwicklung des FloriBot 4.0 in ROS2 dar.

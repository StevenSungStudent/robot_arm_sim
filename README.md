# robotarm-interface

Deze code is gemaakt doormiddel van c++ en Ros2 voor de robot arm simulatie.

Deze opdracht is gemaakt door: Steven & William.

# Uitvoeren van de code
Om het programma te gebruiken moeten de volgende stappen gedaan worden.
Vergeet ook niet om te source'en in elke terminal.

## opstarten van de simulatie
Om de simulatie zelf op te starten moet het volgende commando uitgevoerd worden in de ros workspace:
```
ros2 launch robot_arm_sim display.launch.py
```
## commando's om de arm te bewegen
Hier zijn een paar voorbeeld commando's om te zorgen dat de robot arm beweegt.
```
ros2 topic pub --once /command std_msgs/msg/String '{data: "#2P1700T2000\r"}'
ros2 topic pub --once /command std_msgs/msg/String '{data: "#1P1300T2000\r"}'
ros2 topic pub --once /command std_msgs/msg/String '{data: "#5P1700T2000\r"}'
```
# robotarm-interface

Deze code is gemaakt doormiddel van c++ en Ros2 voor de robot arm simulatie.

Deze opdracht is gemaakt door: Steven & William.

# Uitvoeren van de code
Om het programma te gebruiken moeten de volgende stappen gedaan worden.
Vergeet ook niet om te source'en in elke terminal.

**NOTE** later zal 

## opstarten van de simulatie
Om de simulatie zelf op te starten moet het volgende commando uitgevoerd worden in de ros workspace:
```
ros2 launch robot_arm_sim display.launch.py model:=urdf/lynxmotion_arm.urdf
```
**NOTE** nadat rviz is opgestard MOET je de "joint_state_publisher" window afsluiten, dit wordt later opgelost.

## opstarten van het programma zelf
Om het programma zelf op te starten moet het volgende commando uitgevoerd worden in project folder:
```
ros2 run robot_arm_sim robot_arm_sim
```
## commando's om de arm te bewegen
Hier zijn een paar voorbeeld commando's om te zorgen dat de robot arm beweegt.
```
ros2 topic pub --once /command std_msgs/msg/String '{data: "#2P5T2000\r"}'
```
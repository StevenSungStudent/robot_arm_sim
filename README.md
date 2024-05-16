# robotarm-interface

Deze code is gemaakt doormiddel van c++ en Ros2 voor de robot arm simulatie.

Deze opdracht is gemaakt door: Steven & William.

# Uitvoeren van de code
Om het programma te gebruiken moeten de volgende stappen gedaan worden.

## opstarten van de simulatie
Om de simulatie zelf op te starten moet het volgende commando uitgevoerd worden in de ros workspace:
```
ros2 launch robot_arm_sim display.launch.py model:=urdf/lynxmotion_arm.urdf
```

## opstarten van het programma zelf
Om het programma zelf op te starten moet het volgende commando uitgevoerd worden in project folder:
```
ros2 run robot_arm_sim robot_arm_sim
```
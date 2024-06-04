# robotarm-interface

Deze code is gemaakt doormiddel van c++ en Ros2 voor de robot arm simulatie.

Deze opdracht is gemaakt door: Steven & William.

# Conmpileren van de code
Om het programma te compileren zullen een paar stappen eerst genomen worden.

## Folder structuur
Om de code te compileren moet je een specifieke folder structuur hebben. Je moet een "workspace" forder (naam van de folder maakt niet uit) hebben waarbinnen het project zit, dit ziet eruit als volgt:
```
-workspace 
    -project
```
Binnen die workspace folder zal je de code compileren en uitvoeren. In de "workspace" folder zal dus de build en install folder zitten.

## Compilatie
Het commando voor compilatie is:
```
colcon build --packages-select robot_arm_sim
```

Voor source'en:
```
. install/setup.sh
```

# Uitvoeren van de code
Om het programma te gebruiken moeten de volgende stappen gedaan worden.
Vergeet ook niet om te source'en in elke terminal.

## opstarten van de simulatie
Om de simulatie zelf op te starten moet het volgende commando uitgevoerd worden in de ros workspace:
```
ros2 launch robot_arm_sim display.launch.py
```
## commando's om de arm te bewegen
Het programma neemt commandos die overeen komen met de interface van de daadwerkelijke robot arm.
Het eerste nummer is dus de pin van een servo.
P is de pwm waarde die naar de servo wordt gestuurd, dus de hoek waar de joint naar zal gaan.
T is de tijd in milliseconden die er over gedaan mag worden om de commando uit te voeren.

Voor pins:
```
base:       0
turret:     1
upperarm:   2
forearm:    3
wrist:      4
grippers:   5
```

Voor de commandos moet er een nieuwe ternimal geopend worden (en gescoure'ed).
Hier zijn een paar voorbeeld commando's om te zorgen dat de robot arm beweegt. Wanneer deze commandos uitgevoerd worden zal je zien dat de mok word opgepakt.
```
Open de grippers:
ros2 topic pub --once /command std_msgs/msg/String '{data: "#5P1300T2000\r"}'

Beweeg de upperarm naar beneden:
ros2 topic pub --once /command std_msgs/msg/String '{data: "#2P1700T2000\r"}'

Beweeg de turret naar beneden:
ros2 topic pub --once /command std_msgs/msg/String '{data: "#1P1300T2000\r"}'

Sluit de grippers:
ros2 topic pub --once /command std_msgs/msg/String '{data: "#5P1600T2000\r"}'

Beweeg de upperarm naar bevonen:
ros2 topic pub --once /command std_msgs/msg/String '{data: "#2P1400T2000\r"}'
```

Dit het commando om een joint te laten stoppen, in dit geval joint 2 (upperarm):
```
ros2 topic pub --once /command std_msgs/msg/String '{data: STOP 2\r"}'
```
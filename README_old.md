# Exam_MRCM_2024

## Azioni preliminari

La prima volta che viene fatta la pull del codice, dato che è stato creato un nuovo tipo di "messaggio" (Waypoint.msg), bisogna seguire i seguenti passi:

```sh
cd ~/Scrivania/Exam_MRCM_2024 # fate accesso alla cartella Exam_MRCM_2024
colcon build --packages-select my_msgs
```

**⚠️ Avvertimento:** E' preferibile sempre che questo viene eseguito dopo il colcon build classico, ovvero:

```sh
colcon build --symlink-install
```

Se si hanno dei problemi seguite questi passi (o contattare un esperto):

```sh
rm -rf /home/ddp22/Scrivania/Exam_MRCM_2024/build/my_msgs
colcon build --symlink-install
colcon build --packages-select my_msgs
```

Se si vogliono aggiungere altri "messaggi" personalizzati, interfacce o servizi, questa è la guida da seguire: https://roboticsbackend.com/ros2-create-custom-message/.

# Come eseguire il codice

Una volta eseguite le azioni preliminari, fare i seguenti passi.

## Eseguire il colcon build

```sh
cd ~/Scrivania/Exam_MRCM_2024
colcon build --symlink-install
```

## Eseguire il nodo di navigazione

Apri un terminale es esegui i seguenti comandi:

```sh
cd ~/Scrivania/Exam_MRCM_2024
source install/setup.bash
ros2 run nav_pkg navigation
```

### Esempi di pubblicazione

Se si vuole provare a dare un waypoint di inizio e uno di fine al turtlebot, usate questi comandi:
```sh
ros2 topic pub -1 /start_wp my_msgs/msg/WaypointMsg "{x: 0.0, y: 0.0, direction: 'ovest'}"
ros2 topic pub -1 /end_wp my_msgs/msg/WaypointMsg "{x: 0.0, y: -2.0, direction: 'est'}"
```
Modificare i parametri per provare alrte posizioni.

### Esempi di sottoscrizione

Se si vuole controllare l'output di questo nodo, basta lanciare questo comando SOLO DOPO aver eseguito il nodo:
```sh
ros2 topic echo /goal_reached
```

### Fare navigazione da simulazione da simulazione

Aprire un altro terminale ed eseguire questi comandi SOLO DOPO aver eseguito il navigator:
```sh
cd ~/Scrivania/Exam_MRCM_2024
source install/setup.bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true
```

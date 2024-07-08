# Exam_MRCM_2024

## Clone repository

To clone this repository, you can use the following command in your terminal:
```bash
cd Scrivania/ # You can clone repository in another directory.
git clone https://github.com/DamoraDomenicoPio/Exam_MRCM_2024.git
```

## Setup your code

Perform colcon builds in the cloned directory.
```sh
cd ~/Scrivania/Exam_MRCM_2024
colcon build --symlink-install
colcon build --packages-select my_msgs
```

**⚠️ Warning:** If you have any problems, contact an expert.

## Connect to turtlebot.

If you want connect to turtlebot follow this step:
1. Connecto to the same turtlebot wi-fi
2. run this command
```sh
cd ~/Scrivania/Exam_MRCM_2024
./configure_discovery.sh
# When you run thi last command, insert turtlebot IP when you are asked to enter "RPi4 IP address". Press ENTER for the other requests.
```

3. Check if you are connected to Turtlebot with these commands
```sh
source ~/.bashrc
ros2 daemon stop; ros2 daemon start
ros2 topic list
```

## Load map
1. Now, open a new terminal and launch this commands for loading the map:
```sh
cd ~/Scrivania/Exam_MRCM_2024/src/map
ros2 launch turtlebot4_navigation localization.launch.py map:=diem_map.yaml
```

2. Open a new terminal and launch this commands for launch navigation stack:
```sh
cd ~/Scrivania/Exam_MRCM_2024
ros2 launch turtlebot4_navigation nav2.launch.py
```

3. Open a new terminal and launch this commands to opening Rviz:
```sh
cd ~/Scrivania/Exam_MRCM_2024
ros2 launch turtlebot4_viz view_robot.launch.py
```

## Run on simulation
If you want run the program on the simulator, launch this:
```sh
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true
```

## Run your nodes
### Navigation node
Run this command in another terminal.
```sh
cd ~/Scrivania/Exam_MRCM_2024
source install/setup.bash
ros2 run nav_pkg navigation
```

### QrCode reader
Run this command in another terminal.
```sh
cd ~/Scrivania/Exam_MRCM_2024
source install/setup.bash
ros2 run nav_pkg qr_code_reader
```

## Try your nodes

Try navigation node changing startpoint in code e giving end point through this command on bash:
```sh
cd ~/Scrivania/Exam_MRCM_2024
source install/setup.bash
ros2 topic pub -1 /end_wp my_msgs/msg/WaypointMsg "{x: -3.0, y: -10.0, direction: 0}"
```
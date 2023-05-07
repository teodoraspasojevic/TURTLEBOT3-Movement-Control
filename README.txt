Instrukcije za pokretanje skripti:

1. roscore
2. export TURTLEBOT3_MODEL=burger - pokretanje simulacije
3. roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
4. cdz2src - precica za dolazak do zeljenog direktorijuma
5. rosrun dz2 main.py
7. rosservice call /read_command "{mode: "M"}"
8. export TURTLEBOT3_MODEL=burger
9. roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch --screen
10. rosservice call /read_command "{mode: "A", x_target: 0, y_target: 0, theta_target: 0}"
10. rosservice call /read_command "{mode: "AC", x_target: 0, y_target: 0, theta_target: 0}"

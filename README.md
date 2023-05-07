# TURTLEBOT3-Movement-Control
Control of movement of TURTLEBOT3 robot in Gazebo simulation.

Project description

We create node that uses Subcriber to read position of our robot TURTLEBOT3 in Gazebo simulation from his odometry sensor, reading from odom topic, and Publisher to move robot in simulation sending commands to read_command topic. In nodes, we call functions that generate commands we send to robot, depending of the selected mode of movement, that user selects from service.

Instructions for running the scripts using ROS
1. Run terminal on your Linux PC, and run command roscore.
2. To run simulation, run commands export TURTLEBOT3_MODEL=burger and roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch.
3. Use cd command to navigate to src directory of the project.
4. To run Python scripts use command rosrun, e.g. rosrun dz2 main.py.
5. To move robot using service, run command rosservice, e.g. rosservice call /read_command "{mode: "M"}".
6. To control robot in Manuel mode, usinh keyboard, using service set mode to 'M' (e.g. erosservice call /read_command "{mode: "M"}") and run commands export TURTLEBOT3_MODEL=burger and roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch --screen and move it using keys W, A, S, D, X.
7. To control robot in Auto mode, using service set mode to 'A' and set targets (e.g. rosservice call /read_command "{mode: "A", x_target: 0, y_target: 0, theta_target: 0}").
8. To control robot in Auto mode with constat velocity, using service set mode to 'AC' and set targets (e.g. rosservice call /read_command "{mode: "A", x_target: 0, y_target: 0, theta_target: 0}").

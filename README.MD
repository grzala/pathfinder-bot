Python scripts for a robot that can navigate (globally) and visit a set of predefined points. The robot is capable of finding a path between points using the A* algorithm. It utilises AMCL to reduce the odometry error.

Usage:
Use catkin_make to make the package. Initialize map and params using "roslaunch pbot r.launch" and then use "rosrun pbot navigate.py"to make robot visit points.

To assess the accuracy of the robot, the robot is dropping markers in Rviz, that are helpful for determining if it sticks to the calculated path, etc.

To change the coordinates of points to visit, edit the pbot/r.launch file. If stating position of the robot is edited, the file must also be edited. The same file is also responsible for loading the map and starting the map server.
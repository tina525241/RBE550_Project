# RBE550_Project Package
 
- `roslaunch RBE550_Project turtlebot3_office.launch` to launch burger turtlebot with office map
- `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch` to move robot with keyboard
- To run the navigation interface in rviz `roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml`
   - `map.yaml` is located in the world folder. Move the file to `$home` to run the command or change `map_file` address

-   goals package for moving robot with multiple goals (put the goals package in the same layer with this repository)
-   Code development is based on ([FiorellaSibona](https://github.com/FiorellaSibona/turtlebot3_nav))
-   Use following command to move robot with multiple goals
   -  `roslaunch RBE550_Project turtlebot3_office.launch `
   -  `roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml `
   -  `roslaunch goals movebase_seq.launch `
  



-----edit my world-----
1. `cd ./catkin_ws/src/RBE550_Project/world`
2. `gazebo world_street.world`

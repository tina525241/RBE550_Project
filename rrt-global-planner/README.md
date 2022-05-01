# rrt*-global-planner
A [Rapidly Exploring Random Trees (RRT)](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree) global path planner plugin for ROS.

## Demo
![](https://github.com/tina525241/RBE550_Project/blob/rrtStar_attempt/rrt-global-planner/assets/rrtStar%20Demo.gif)

## Install Dependancies and Build
This package was created to run in:
ROS Noetic 

Turtlebot 3 is needed to run this plugin. 
These instuctions come from "How to Launch the Turtlebot3 Simulation With ROS", by Automatic Addison (https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/).
```bash
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/catkin_ws && catkin_make
```
Then it is necessary to establish the default model used, the 'burger' model.
```bash
gedit ~/.bashrc
```
Add the following line at the end of the file: 'export TURTLEBOT3_MODEL=burger'
Save the file, close it and continue by reloading the file and finally retrieving the Simulation files.
```
source ~/.bashrc

cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws && catkin_make

```
For the dependencies of this package,
```bash
cd ~/catkin_ws/src
```
Copy the `rrt-global-planner` folder to this location.
```
cd .. && rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## Usage
Within the [move\_base](https://wiki.ros.org/move_base) node in your launch file, set the `base_global_planner` parameter to `global_planner/RRTGlobalPlanner` and load the required parameters.
```xml
<param name="base_global_planner" value="global_planner/RRTGlobalPlanner"/>
<rosparam file="$(find rrt-global-planner)/params/rrt_global_planner.yaml" command="load" />
```
In the `amcl.launch` file (`turtlebot3/turtlebot3_navigation/launch`), change the `initial_pose_x` to -2.0 and the `initial_pose_y` to -0.5.
```
  <arg name="initial_pose_x" default="-2.0"/>
  <arg name="initial_pose_y" default="-0.50"/>
```
In the `turtlebot3_world.launch` file (`turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/launch`), change the `gui` default value to `false`.
```
    <arg name="gui" value="false"/>
```
After launching the system, when you set a `move_base/goal` using RViz's `2D Nav Goal` or with an action client, the `RRTGlobalPlanner` will be called. The global path will be published as a topic for visualization. Optionally, a visualization of the full RRT constructed for path planning will be published.

`RRTGlobalPlanner`'s output can be visualized in RViz. To see the global path, add a `Path` display and subscribe to `~/move_base/RRTGlobalPlanner/plan`. To see the full tree (`viz_tree` must be true), add a `Marker` display and subscribe to `~/move_base/RRTGlobalPlanner/tree`.

## ROS API
### Published Topics
`~/move_base/RRTGlobalPlanner/plan` ([nav\_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))
- The global path constructed by the planner. Used for visualization purposes.

`~/move_base/RRTGlobalPlanner/tree` ([visualization\_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))
- Visualization of full tree built during planning process.

### Subscribed Topics
None.

### Parameters
`~/move_base/RRTGlobalPlanner/goal_tol` (`double`, default: 0.05)
- Cartesian goal tolerance to be achieved by the global planner.

`~/move_base/RRTGlobalPlanner/K_in` (`int`, default: 4000)
- Maximum number of iterations to attempt to find a plan.

`~/move_base/RRTGlobalPlanner/d` (`double`, default: 0.2)
- Distance to extend tree per iteration.

`~/move_base/RRTGlobalPlanner/bias` (`int`, default: 7)
- The percent chance to choose the goal state as the Random State.

`~/move_base/RRTGlobalPlanner/viz_tree` (`bool`, default: false)
- Whether to publish full tree on `~/move_base/RRTGlobalPlanner/tree` topic for visualization purposes after planning success. 

### Services
None.

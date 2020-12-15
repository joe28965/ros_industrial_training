# ROS Industrial Training

## Setup environment for Turtlebot3
Add the following two lines to `.bashrc`

```
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_simulation_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
```

## Start the default Turtlebot simulation
Source the turtlebot simulation workspace
`source ~/turtlebot3_simulation_ws/devel/setup.bash`

Start the simulation in the turtlebot world.
`$ roslaunch turtlebot3_gazebo turtlebot3_world.launch`

Navigate with default map
`$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch`

# AR Demonstration
Clone the repository:

```
$ cd
$ mkdir -p ros_ar_nav_ws/src
$ cd ros_ar_nav_ws/src
$ git clone https://github.com/joe28965/ros_industrial_training.git
$ cd ..
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin build
$ source devel/setup.bash
```
Move the models folder from the repo to ~/.gazebo/ (merge in the existing one)

start simulation (in seperate terminals):
```
$ roslaunch ros_industrial_training turtlebot3_world_markers.launch
$ roslaunch ros_industrial_training turtlebot3_startup.launch
$ roslaunch ros_industrial_training ar_navigation.launch
```

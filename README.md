# UV_BOT
Project of a sanitizer robot developed for my Autonomous and Mobile Robotics class.  
This project aims at developing a robot that can navigate a simulated environment while simultaneously generating an irradiation map of the rooms it has to move trough.  
This package must be git cloned in your ROS workspace `src` folder.
A report of the project is included in the files in pdf format.

### Other packages required: 
Turtlebot3 complete install with added packages  
EIGEN : `sudo apt-get install libeigen3-dev`  
ANYbotics grid_map : `sudo apt-get install ros-$ROS_DISTRO-grid-map`  
  
### Important commands:
1) remember to `$ source /opt/ros/noetic/setup.bash`
2) if it is a first install on new machine, delete `build` and `devel` folders
3) `$ catkin_make` for initial build, `$ catkin_make -DCMAKE_BUILD_TYPE=Release` for release (faster computation, longer compile times)
4) __ALWAYS__ `$ source devel/setup.bash`

### How to use:

1) Map generation: `$ roslaunch sanitizer_launch map_generation.launch`
2) Sanification: `$ roslaunch sanitizer_launch sanitizer_bot.launch`

### Images: 

![map generation](img/explore_lite.png)
map_generation

![sanitizer node in function](img/irradiated_map1.png)
sanitizer_bot 

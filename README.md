# UAV Path Planning
Implementation of path planning and trajectory algorithm for Unmanned Aerial Vehicle

## Dependency files
The Complete ROS packages for the simulation can be found on link below
https://drive.google.com/open?id=1Hc5TFrbhHfVVUc1gprNVxWLLa-h_yjMV

### Modification from dependency folder
- uav_pathplanning/src/hector_quadrotor/hector_quadrotor_reference/src/main.cpp
  - turning off motor when the robot height position = 0 after over 100 lines
  - iterate obs files to add up to 5 obstacles into the vicon room
- uav_pathplanning/src/hector_gazebo/hector_gazebo_worlds/worlds/viconRoom.world
  added `unit_cylinder_4` and `unit_cylinder_5` to viconRoom world file to cater up to 5 obstacles.
### How To
- replace hector `hector_gazebo/hector_gazebo_worlds` and `hector_quadrotor/hector_quadrotor_reference` with the folder provided in this repo
- compile the workspace with `catkin_make`

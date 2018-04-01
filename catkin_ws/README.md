# HWATG ROS WORKSPACE

All of the ROS nodes from our robot live in here. It's pretty messy, and will only work properly on our robot (not sure where that ended up...)

- ```robot_stack``` is the competition code. It launches a bunch of nodes for vision, PID control, servo actuation, and managing the robot's state machine.
- ```convert_map_to_og/```: A ROS package for converting the text file maps given to us into occupancy grids.
- ```kinect_slam_2d/```: Code for doing 2d localization using a Microsoft Kinect. We didn't end up using this in competition because it didn't work well. 2D localization breaks down when the map you're in has a lot of symmetry (i.e an octagon in our case).

The repository for demonstration purposes only.

# Architecture 
We use BFS to find reachable sells. Once selected, one's coordinates are sent to Nav2 planner in order to compute safe path. When reached - process repeats. 
This navigation setup is based on LD19 2-D Lidar and ROS slam_toolbox which constructs the map of our surroundings.

If, along the way, the garbage is detected, navigation interrupts and control_node takes over(We use NanoTrack to lock on to the target, YOLO model runs once in certain period in order to confirm detection). If the rover managed to approach the garbage at desired distance(e.g 0.4 m), we initiate robotic arm action through ROS2 service. 3-D coordinates of the object extracted from depth image and converted into arm's frame. Then, arms tries to pick up the object and place back into the "trunk" (check the video).

More specifically, YOLO model and depth-coordinates extraction run in different nodes(inference_node and visualizer_node respectively). control_node is a state machine to command robot's movement once the object is detected.

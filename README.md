# Robotics-Final-Project
The final project of the Robotics course. A robot has to find it's way out of a maze with the VFH algorithm. This robot should also detect lanes and traffic signs in a city setting and act accordingly.

## Requirements:
- [ROS Noetic](http://wiki.ros.org/noetic)
- [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3), [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs) and [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations), [turtlebot3_autorace_2020](https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020.git)
- [numpy](https://numpy.org/)
- [ultralytics](https://docs.ultralytics.com/)
- [opencv-python](https://pypi.org/project/opencv-python/)

## First Phase: The VFH Pathfinding Algorithm
In this phase, our goal is to implement *pathfinding* and *obstacle avoidance* using the **Vector Field Histogram** (VFH) algorithm. The robot (which is a `turtlebot3` robot) should start at the starting position of $(0, 0)$ in the world of `updated_maze.world` located in the [/worlds](https://github.com/rzninvo/robotics_final_project/tree/main/worlds) folder. The objective of this robot is to avoid all the obstacles in the maze and find it's way to the end of the maze which is at the goal position of $(13, 7)$.   
This package implements the VFH Algorithm based on [this paper](http://www-personal.umich.edu/~ykoren/uploads/The_Vector_Field_HistogramuFast_Obstacle_Avoidance.pdf).

### Our changes to the VFH Algorithm:
In this package, some changes were done by us to further optimize the algorithm. Here is a brief explanation:
1. **Changes to the Confidence Calculation**:   
Since in our implementation, the **Lidar Scanner** only gives us the distance to any object surounding the robot and not it's confidence of being an actual object, we implemented our own confidence formula:   
$c_\theta = |\frac{d}{ws} - 1|$   
Where $c_\theta$ is the confidence of the corresponding laser reading angle, $d$ is the laser reading distance, and $ws$ is our window size.   
This formula makes it so that the nearest object to the robot is close to $1$ and the farthest object in the **Active Window** is close to $0.5$. This formula has proven to have a better **Polar Density Histogram** than setting it to the default confidence of $1$ for every reading in the **Active Window**.
3. **Filtering out certain angles**:   
In some situations where there is no possible **Candidate Valley** other than the angles between $(135\degree, 225\degree)$, the robot would choose these valleys as it's nearest valley to the target and attempt to make a $180\degree$ turn. To avoid these situations, we have filtered out the aforementioned angles and don't recognize them as candidate valleys.

### Components
1. **The [VFH Algorithm Node](https://github.com/rzninvo/robotics_final_project/blob/main/src/vfh_algorithm_node.py):**   
This node acts as a service node, plotting the **Polar Density Histogram** of the robot and constantly giving our robot it's best rotating angle to the goal direction while avoiding obstacles using the VFH algorithm. It's message type is found [here](https://github.com/rzninvo/robotics_final_project/blob/main/srv/vfh_planner.srv).   
2. **The [VFH Pathfinder Node](https://github.com/rzninvo/robotics_final_project/blob/main/src/vfh_pathfinder_node.py):**   
This node acts as the control node, while having a proxy to the `vfh_planner_service` and feeding on it's calculated output angle and $h_c$. This calculated output angle acts as the error of a **PID** control for the angular velocity of our robot. The output $h_c$ acts as the control for the robot's linear velocity based on the VFH paper.

### How do I build this?
1. Install the requirements.
2. Clone this package beside your `turtlebot3` packages:   
```console
foo@bar:~/catkin/src$ git clone https://github.com/rzninvo/robotics_final_project.git
```
3. Navigate to the root directory of your `catkin` workspace.
4. Do `catkin_make`:   
```console
foo@bar:~/catkin$ catkin_make
```

### How do I run this?
1. Navigate to the root directory of your `catkin` workspace.
2. Source your workspace:   
```console
foo@bar:~/catkin$ . devel/setup.bash
```
3. Launch the provided launch file:   
```console
foo@bar:~/catkin$ roslaunch robotics_final_project vfh_planning.launch
```

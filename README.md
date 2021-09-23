# CS4278/CS5478 Lab 1 Motion Planning for Mobile Robots


## Simulator Installation and Setup
In Lab 1pre, you have already installed ROS Kinetic. The *FoodTutrle* simulator for this lab is adapted from  the  [ROS TurtleBot Stage](http://wiki.ros.org/turtlebot_stage) package.
Install TurtleBot Stage:
```
sudo apt install ros-kinetic-turtlebot-stage
sudo apt install ros-kinetic-joint-state-publisher-gui
```

Clone this repo and setup the environment:
```
catkin_make
source devel/setup.bash
```

Check your setup:
```
roslaunch planner turtlebot_in_stage.launch
```
You should see RViz and ROS Stage. 

## Code Overview
The FoodTurtle system consists of three components: simulator, planner, and controller. The code in this repo provides most of these components, including the required functions to set up ROS nodes, data communication, and robot control in the simulator. You will implement the planning algorithms and collision avoidance capability by filling in the template marked with `# TODO: FILL ME!` in base_planner.py. We  recommend that you  use base_planner.py as a base class and then impelement your planners as derived classes.

## Code Execution
Launch the simulator and execute the planner: 
```
roscd planner/src
sh run.sh [your map name] start_x start_y start_theta
python your_planner.py --goal 'goal_x,goal_y' --com use_com_1_map_or_not
```

For example, load *map1.png* and set the robot start pose as (x=1, y=1, θ=0): 
```
roscd planner/src
sh run.sh map1.png 1 1 0
```

Set the robot goal as (x'=5, y'=5) and run the planner:
```
python your_planner.py --goal '5,5' --com 0
```
The flag `--com`  indicates whether the COM1 map is used, as it requires a special set of environment parameters. 

## Debugging and Visualization
For visualization,  we recommend  ROS stage. RViz provides 2.5-D visualization, but may be noisy  due to ROS asynchronous communication delays.

## Submission

A [dataset](./src/planner/maps/) of 5 maps are provided for evaluation, including 4 handcrafted maps (map1.png to map4.png) and a simplified COM1 level-1 floorplan (com1.jpg).  Each map has a corresponding list of testcases, with the goals specified [here](./files/goals.json). The robot always starts with pose (1, 1, 0).

Evaluate your planners on all testcases under the three models, DSDA, CSDA, and DSPA. For DSDA and CSDA,  save the control actions in a `txt` files and name it {task}_{map}_{goal}.txt. For DSPA,  save the MDP control policy in a  `json` files and name it {task}_{map}_{goal_x}_{goal_y}.json. 
For example, 
- DSDA_map2_5_5.txt for DSDA on map2.png with [5, 5] as the goal.
- CSDA_map3_9_9.txt for CSDA on map3.png with [9, 9] as the goal.
- DSPA_com1_43_10.json for the MDP policy on com1.jpg with [43, 10] as the goal.

You may find the functions to save the results in base_planner.py. Some example control files can be found [here](./files/).

Finally, submit a single zip archive and name it MatricNo-Lab1.zip. It contains 3 directories:
- `Code`: your code
- `Controls`: the control files for all testcases
- `Report`: your report

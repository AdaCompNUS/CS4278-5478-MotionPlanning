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
The FoodTurtle system consists of three components: simulator, planner, and controller. The code in this repo provides most of these components, including the required functions to set up ROS nodes, data communication, and robot control in the simulator. You will implement the planning algorithms and collision avoidance capability by filling in the template marked with `# TODO: FILL ME!` in `base_planner.py`. We  recommend that you  use `base_planner.py` as a base class and then impelement your planners as derived classes.

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

We provide you with 5 maps, including 4 handcrafted maps (map1.png to map4.png) and an illustrative COM1 level 1 floorplan (com1.jpg). You can find them [here](./src/planner/maps/). Each map has a list of corresponding testcases, with the goals specified [here](./files/goals.json). For all cases, we assume the robot starts with pose (1, 1, 0).

You should implement the planners, test them, generate controls for each testcase, and submit all of them. For DSDA and CSDA (task 1 and task 2),  save them in `.txt` files. For DSPA MDP policy (task 3),  save it into a json file. We have provided functions in base_planner.py.

The naming should follow `{task}_{map}_{goal}.txt` for task 1 and task 2; `{task}_{map}_{goal_x}_{goal_y}.json` for task 3. For example, 

- `1_map2_5_5.txt` for the discrete planner on map2.png with [5, 5] as the goal.
- `2_map3_9_9.txt` for the continuous planner on map3.png with [9, 9] as the goal.
- `3_com1_43_10.json` for the mdp policy on com1.jpg with [43, 10] as the goal.

Some example control files can be found [here](./files/).

In summary, you should submit a zip file named MatricNo-Lab1.zip with following folders:
- Code: your code
- Controls: the control files for each map and each goal
- Report: your report

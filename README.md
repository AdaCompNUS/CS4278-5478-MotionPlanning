# CS4278/CS5478 Lab 1 Motion Planning for Mobile Robots


## Simulator Installation and Setup

In Lab 1pre, you have already installed ROS Kinetic. The *FoodTutrle* simulator
for this lab is adapted from the [ROS TurtleBot
Stage](http://wiki.ros.org/turtlebot_stage) package. Install TurtleBot Stage:

```
sudo apt install ros-kinetic-turtlebot-stage
sudo apt install ros-kinetic-joint-state-publisher-gui
```

Clone this repository and set up the environment:
```
git clone https://github.com/AdaCompNUS/CS4278-5478-MotionPlanning
cd CS4278-5478-MotionPlanning
catkin_make
source devel/setup.bash
```

Check your setup:
```
roslaunch planner turtlebot_in_stage.launch
```
You should see RViz and ROS Stage. 

## Code Overview

The FoodTurtle system consists of three components: the simulator, the planner,
and the controller. The code in this repository provides most of these
components, including the necessary functions for setting up ROS nodes in the
simulator, data communication, and robot control. You will implement the
planning algorithm and some robot interfaces by filling in the template labeled
`# TODO: FILL ME!` in `base_planner.py`. We recommend that you use
`base_planner.py` as a base class and then implement your planners as derived
classes.

## Code Execution
Launch the simulator and execute the planner: 
```
roscd planner/src
sh run.sh [your map name] start_x start_y start_theta
python your_planner.py --goal 'goal_x,goal_y' --com use_com_1_map_or_not
```

For example, load `map1.png` and set the robot start pose as (x=1, y=1, θ=0): 
```
roscd planner/src
sh run.sh map1 1 1 0
```

Set the robot goal as (x=5, y=5) and run the planner:
```
python your_planner.py --goal '5,5' --com 0
```
The flag `--com` indicates whether the COM1 map is used, as it requires a special set of environment parameters. 

Without adding any implementation, you should see the robot move forward by two
tiles and then turn left.

## Debugging and Visualization
For visualization, we recommend the ROS Stage. RViz provides 2.5-D
visualization, but may be noisy due to the asynchronous communication delay of
ROS.

## Performance Evaluation
A [dataset](./src/planner/maps/) provides 5 maps for evaluation, including 4
hand-crafted maps (`map1.png` to `map4.png`) and a simplified COM1 level-1 floor
plan (`com1building.png`). Each map has a corresponding list of test cases with goals
specified [here](./files/goals.json). The robot always starts with the pose `(1,
1, 0)`.

Evaluate your planners in all test cases under the three models, DSDA, CSDA and
DSPA. For DSDA and CSDA, save the control actions in a `txt` file and name it
`{task}_{map}_{goal_x}_{goal_y}.txt`. For DSPA, save the MDP control policy in a
`json` file and name it `{task}_{map}_{goal_x}_{goal_y}.json`.


For example, 
- `DSDA_map2_5_5.txt` for DSDA on `map2.png` with `[5, 5]` as the goal.
- `CSDA_map3_9_9.txt` for CSDA on `map3.png` with `[9, 9]` as the goal.
- `DSPA_com1building_43_10.json` for the MDP policy on com1building.png with `[43, 10]` as the
  goal.

You can find the functions to save the results in base_planner.py. Some examples
of control files can be found [here](./files/).

## Submission
Submit a single zip archive named `{MatricNo}-Lab1.zip`, which contains 3 directories:
- `Code`: your code
- `Controls`: the control files for all testcases
- `Report`: your report

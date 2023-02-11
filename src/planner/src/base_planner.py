#!/usr/bin/env python
import math
from math import *
import json
import copy
import argparse

import rospy
import numpy as np
from geometry_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *
from const import *

from planner.msg import ContinuousAction, DiscreteAction, ContinuousState, DiscreteState
from planner.srv import (
    DiscreteActionSequenceExec,
    ContinuousActionSequenceExec,
    DiscreteActionStochasticExec,
)

ROBOT_SIZE = 0.2552  # width and height of robot in terms of stage unit


def dump_action_table(action_table, filename):
    """dump the MDP policy into a json file

    Arguments:
        action_table {dict} -- your mdp action table. It should be of form {(1,2,0): (1, 0), ...}
        filename {str} -- output filename
    """
    tab = dict()
    for k, v in action_table.items():
        key = [str(int(i)) for i in k]
        key = ",".join(key)
        tab[key] = v

    with open(filename, 'w') as fout:
        json.dump(tab, fout)


class Planner:
    def __init__(self, world_width, world_height, world_resolution, inflation_ratio=3):
        """init function of the base planner. You should develop your own planner
        using this class as a base.

        For standard mazes, width = 200, height = 200, resolution = 0.05. 
        For COM1 map, width = 2500, height = 983, resolution = 0.02

        Arguments:
            world_width {int} -- width of map in terms of pixels
            world_height {int} -- height of map in terms of pixels
            world_resolution {float} -- resolution of map

        Keyword Arguments:
            inflation_ratio {int} -- [description] (default: {3})
        """
        self.map = None
        self.pose = None
        self.goal = None
        self.action_seq = None  # output
        self.aug_map = None  # occupancy grid with inflation
        self.action_table = {}

        self.world_width = world_width
        self.world_height = world_height
        self.resolution = world_resolution
        self.inflation_ratio = inflation_ratio
        self.setup_map()
        rospy.sleep(1)

    def setup_map(self):
        """Get the occupancy grid and inflate the obstacle by some pixels.

        You should implement the obstacle inflation yourself to handle uncertainty.
        """
        # Hint: search the ROS message defintion of OccupancyGrid
        occupancy_grid = rospy.wait_for_message('/map', OccupancyGrid)
        self.map = occupancy_grid.data

        # TODO: FILL ME! implement obstacle inflation function and define self.aug_map = new_mask

        # you should inflate the map to get self.aug_map
        self.aug_map = copy.deepcopy(self.map)

    def _get_goal_position(self):
        goal_position = self.goal.pose.position
        return (goal_position.x, goal_position.y)

    def set_goal(self, x, y, theta=0):
        """set the goal of the planner

        Arguments:
            x {int} -- x of the goal
            y {int} -- y of the goal

        Keyword Arguments:
            theta {int} -- orientation of the goal; we don't consider it in our planner (default: {0})
        """
        a = PoseStamped()
        a.pose.position.x = x
        a.pose.position.y = y
        a.pose.orientation.z = theta
        self.goal = a

    def generate_plan(self, init_pose):
        """TODO: FILL ME! This function generates the plan for the robot, given
        an initial pose and a goal pose.

        You should store the list of actions into self.action_seq, or the policy
        into self.action_table.

        In discrete case (task 1 and task 3), the robot has only 4 heading directions
        0: east, 1: north, 2: west, 3: south

        Each action could be: (1, 0) FORWARD, (0, 1) LEFT 90 degree, (0, -1) RIGHT 90 degree

        In continuous case (task 2), the robot can have arbitrary orientations

        Each action could be: (v, \omega) where v is the linear velocity and \omega is the angular velocity
        """
        self.action_seq = []

    def collision_checker(self, x, y):
        """TODO: FILL ME!
        You should implement the collision checker.
        Hint: you should consider the augmented map and the world size
        
        Arguments:
            x {float} -- current x of robot
            y {float} -- current y of robot
        
        Returns:
            bool -- True for collision, False for non-collision
        """

        return False

    def motion_predict(self, x, y, theta, v, w, dt=0.5, frequency=10):
        """Predict the next pose of the robot given controls. Returns None if
        the robot collide with the wall.

        The robot dynamics is provided in the assignment description.

        Arguments:
            x {float} -- current x of robot
            y {float} -- current y of robot
            theta {float} -- current theta of robot
            v {float} -- linear velocity 
            w {float} -- angular velocity

        Keyword Arguments:
            dt {float} -- time interval. DO NOT CHANGE (default: {0.5})
            frequency {int} -- simulation frequency. DO NOT CHANGE (default: {10})

        Returns:
            tuple -- next x, y, theta; return None if has collision
        """
        num_steps = int(dt * frequency)
        dx = 0
        dy = 0
        for i in range(num_steps):
            if w != 0:
                dx = - v / w * np.sin(theta) + v / w * \
                    np.sin(theta + w / frequency)
                dy = v / w * np.cos(theta) - v / w * \
                    np.cos(theta + w / frequency)
            else:
                dx = v*np.cos(theta)/frequency
                dy = v*np.sin(theta)/frequency
            x += dx
            y += dy

            if self.collision_checker(x, y):
                return None
            theta += w / frequency
        return x, y, theta

    def discrete_motion_predict(self, x, y, theta, v, w, dt=0.5, frequency=10):
        """Discrete version of the motion predict.

        Note that since the ROS simulation interval is set to be 0.5 sec and the
        robot has a limited angular speed, to achieve 90 degree turns, we have
        to execute two discrete actions consecutively. This function wraps the
        discrete motion predict.

        Please use it for your discrete planner.

        Arguments:
            x {int} -- current x of robot
            y {int} -- current y of robot
            theta {int} -- current theta of robot
            v {int} -- linear velocity
            w {int} -- angular velocity (0, 1, 2, 3)

        Keyword Arguments:
            dt {float} -- time interval. DO NOT CHANGE (default: {0.5})
            frequency {int} -- simulation frequency. DO NOT CHANGE (default: {10})

        Returns:
            tuple -- next x, y, theta; return None if has collision
        """
        w_radian = w * np.pi/2
        first_step = self.motion_predict(x, y, theta*np.pi/2, v, w_radian)
        if first_step:
            second_step = self.motion_predict(
                first_step[0], first_step[1], first_step[2], v, w_radian)
            if second_step:
                return (round(second_step[0]), round(second_step[1]), round(second_step[2] / (np.pi / 2)) % 4)
        return None


class RobotClient:
    """A class to interface with the (simulated) robot.

    You can think of this as the "driver" program provided by the robot manufacturer ;-)
    """

    def __init__(self):
        self._cstate = None
        self.sb_cstate = rospy.Subscriber(
            "/lab1/continuous_state", ContinuousState, self._cstate_callback
        )
        self._dstate = None
        self.sb_dstate = rospy.Subscriber(
            "/lab1/discrete_state", DiscreteState, self._dstate_callback
        )

    def _cstate_callback(self, msg):
        """Callback of the subscriber."""
        self._cstate = msg

    def get_current_continuous_state(self):
        """Get the current continuous state.

        Returns:
            tuple -- x, y, \theta, as defined in the instruction document.
        """
        return (self._cstate.x, self._cstate.y, self._cstate.theta)

    def _dstate_callback(self, msg):
        self._dstate = msg

    def get_current_discrete_state(self):
        """Get the current discrete state.

        Returns:
            tuple -- x, y, \theta, as defined in the instruction document.
        """
        return (self._dstate.x, self._dstate.y, self._dstate.theta)

    def _d_from_target(self, target_pose):
        """Compute the distance from current pose to the target_pose.

        Arguments:
            pose {list} -- robot pose

        Returns:
            float -- distance to the target_pose
        """
        pose = self.get_current_continuous_state()
        return math.sqrt(
            (pose[0] - target_pose[0]) ** 2 + (pose[1] - target_pose[1]) ** 2
        )

    def is_close_to_goal(self, goal):
        """Check if close enough to the given goal.

        Arguments:
            pose {list} -- robot post

        Returns:
            bool -- goal or not
        """
        return self._d_from_target(goal) < 0.3

    def publish_discrete_control(self, action_seq, goal):
        """Publish the discrete controls"""
        proxy = rospy.ServiceProxy(
            "/lab1/discrete_action_sequence",
            DiscreteActionSequenceExec,
        )
        plan = [DiscreteAction(action) for action in action_seq]
        proxy(plan)
        assert self.is_close_to_goal(goal), "Didn't reach the goal."

    def publish_continuous_control(self, action_seq, goal):
        """Publish the continuous controls.

        TODO: FILL ME!

        You should implement the ROS service request to execute the motion plan.

        The service name is /lab1/continuous_action_sequence

        The service type is ContinuousActionSequenceExec

        Checkout the definitions in planner/msg/ and planner/srv/
        """
        pass
        assert self.is_close_to_goal(goal)

    def execute_policy(self, action_table, goal):
        """Execute a given policy in MDP.

        Due to the stochastic dynamics, we cannot execute the motion plan
        without feedback. Hence, every time we execute a discrete action, we
        query the current state by `get_current_discrete_state()`.

        You don't have to worry about the stochastic dynamics; it is implemented
        in the simulator. You only need to send the discrete action.
        """
        # TODO: FILL ME!
        # Instantiate the ROS service client
        # Service name: /lab1/discrete_action_stochastic
        # Service type: DiscreteActionStochasticExec
        # Checkout the definitions in planner/msg/ and planner/srv/
        while not self.is_close_to_goal(goal):
            current_state = self.get_current_discrete_state()
            action = action_table[current_state]
            # TODO: FILL ME!
            # Put the action into proper ROS request and send it
        rospy.sleep(1)
        assert self.is_close_to_goal(goal)


if __name__ == "__main__":
    # TODO: You can generate and save the plan using the code below
    rospy.init_node('planner')
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal', type=str, default='1,8',
                        help='goal position')
    parser.add_argument('--com', type=int, default=0,
                        help="if the map is com1 map")
    args = parser.parse_args()

    try:
        goal = [int(pose) for pose in args.goal.split(',')]
    except:
        raise ValueError("Please enter correct goal format")

    if args.com:
        width = 2500
        height = 983
        resolution = 0.02
    else:
        width = 200
        height = 200
        resolution = 0.05

    robot = RobotClient()
    inflation_ratio = 3  # TODO: You should change this value accordingly
    planner = Planner(width, height, resolution, inflation_ratio=inflation_ratio)
    planner.set_goal(goal[0], goal[1])
    if planner.goal is not None:
        planner.generate_plan(robot.get_current_discrete_state())

    # Let's try executing a hard-coded motion plan!
    # Forward! Forward! Turn left!
    # You should see an AssertionError since we didn't reach the goal.
    mock_action_plan = [(1, 0), (1, 0), (0, 1)]
    robot.publish_discrete_control(mock_action_plan, goal)
    # TODO: FILL ME!
    # After you implement your planner, send the plan/policy generated by your
    # planner using appropriate execution calls.
    # e.g., robot.publish_discrete_control(planner.action_seq, goal)

    # save your action sequence
    result = np.array(planner.action_seq)
    np.savetxt("actions_continuous.txt", result, fmt="%.2e")

    # for MDP, please dump your policy table into a json file
    # dump_action_table(planner.action_table, 'mdp_policy.json')

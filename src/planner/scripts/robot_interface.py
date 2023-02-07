#!/usr/bin/env python
import time
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from planner.msg import DiscreteState, ContinuousState
from planner.srv import (
    DiscreteActionStochasticExec,
    DiscreteActionStochasticExecResponse,
    DiscreteActionSequenceExec,
    DiscreteActionSequenceExecResponse,
    ContinuousActionSequenceExec,
    ContinuousActionSequenceExecResponse,
)


def create_control_msg(x, y, z, ax, ay, az):
    """a wrapper to generate control message for the robot.

    Arguments:
        x {float} -- vx
        y {float} -- vy
        z {float} -- vz
        ax {float} -- angular vx
        ay {float} -- angular vy
        az {float} -- angular vz

    Returns:
        Twist -- control message
    """
    message = Twist()
    message.linear.x = x
    message.linear.y = y
    message.linear.z = z
    message.angular.x = ax
    message.angular.y = ay
    message.angular.z = az
    return message


class Lab1Interface:
    def __init__(self):
        self.pose = None
        self._pub_ctrl = rospy.Publisher(
            "/mobile_base/commands/velocity", Twist, queue_size=10
        )
        self._pub_dstate = rospy.Publisher(
            "/lab1/discrete_state", DiscreteState, queue_size=1
        )
        self._pub_cstate = rospy.Publisher(
            "/lab1/continuous_state", ContinuousState, queue_size=1
        )
        self._discrete_action_server = rospy.Service(
            "/lab1/discrete_action_sequence",
            DiscreteActionSequenceExec,
            self._discrete_plan_cb,
        )
        self._continuous_action_server = rospy.Service(
            "/lab1/continuous_action_sequence",
            ContinuousActionSequenceExec,
            self._continuous_plan_cb,
        )
        self._discrete_stochastic_action_server = rospy.Service(
            "/lab1/discrete_action_stochastic",
            DiscreteActionStochasticExec,
            self._discrete_stochastic_cb,
        )

        self.sb_pose = rospy.Subscriber(
            "/base_pose_ground_truth", Odometry, self._pose_callback
        )

    def _continuous_plan_cb(self, req):
        for action in req.plan:
            msg = create_control_msg(action.v, 0, 0, 0, 0, action.w)
            self._pub_ctrl.publish(msg)
            rospy.sleep(0.6)
        resp = ContinuousActionSequenceExecResponse()
        resp.done = True
        return resp

    def _discrete_plan_cb(self, req):
        for action in req.plan:
            forward, turn = action.action
            msg = create_control_msg(forward, 0, 0, 0, 0, turn * np.pi / 2)
            self._pub_ctrl.publish(msg)
            rospy.sleep(0.6)
            self._pub_ctrl.publish(msg)
            rospy.sleep(0.6)
        resp = DiscreteActionSequenceExecResponse()
        resp.done = True
        return resp

    def _discrete_stochastic_cb(self, req):
        forward, turn = req.action.action
        if int(forward) == 1 and int(turn) == 0:
            r = np.random.rand()
            if r < 0.9:
                forward = 1
                turn = 0
            elif r < 0.95:
                forward = np.pi / 2
                turn = 1
            else:
                forward = np.pi / 2
                turn = -1
        msg = create_control_msg(forward, 0, 0, 0, 0, turn * np.pi / 2)
        self._pub_ctrl.publish(msg)
        rospy.sleep(0.6)
        self._pub_ctrl.publish(msg)
        rospy.sleep(0.6)
        time.sleep(1)
        resp = DiscreteActionStochasticExecResponse()
        resp.done = True
        return resp

    def _pose_callback(self, msg):
        self.pose = msg
        x = self.pose.pose.pose.position.x
        y = self.pose.pose.pose.position.y
        orientation = self.pose.pose.pose.orientation
        ori = [orientation.x, orientation.y, orientation.z, orientation.w]

        theta = np.arctan2(
            2 * (ori[0] * ori[1] + ori[2] * ori[3]), 1 - 2 * (ori[1] ** 2 + ori[2] ** 2)
        )
        cstate = ContinuousState(x=x, y=y, theta=theta)

        def rd(x):
            return int(round(x))

        dstate = DiscreteState(x=rd(x), y=rd(y), theta=(rd(theta / (np.pi / 2)) % 4))
        self._pub_cstate.publish(cstate)
        self._pub_dstate.publish(dstate)


if __name__ == "__main__":
    rospy.init_node("lab1_robot_interface")
    interface = Lab1Interface()
    rospy.loginfo("Robot action interface ready!")
    rospy.spin()

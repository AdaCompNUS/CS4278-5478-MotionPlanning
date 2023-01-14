"""A script that receives user commands to steer the turtlesim robot."""
import sys
import time

import numpy as np
import rospy
import std_srvs.srv
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class Controller:
    """A simple controller/policy."""

    def __init__(self, x_goal, y_goal):
        self._goal = np.array([x_goal, y_goal])

    def __call__(self, x, y, theta):
        e = self._goal - np.array([x, y])
        e_dist = np.linalg.norm(e)
        e_theta = np.arctan2(e[1], e[0]) - theta
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))
        # Treat distance and angle as two independent first-order systems
        v = min(e_dist, 1)  # clamp/saturation
        w = 2 * e_theta
        return v, w

    @property
    def goal(self):
        return self._goal


class PositionSubscriber:
    """Subscriber to the robot's sensor."""

    def __init__(self):
        # Instantiate the ROS subscriber
        self._listener = rospy.Subscriber("/turtle1/pose", Pose, self._callback)

    def _callback(self, msg):
        """Callback function to store relevant data."""
        self.data = (msg.x, msg.y, msg.theta)


class CmdPublisher:
    """Publisher to the robot's actuator."""

    def __init__(self):
        # Instantiate the ROS publisher
        self._pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)

    def __call__(self, v, w):
        """Format and then publish the ROS message."""
        # Put the data into the proper ROS message format
        msg = Twist()  # A ROS message
        msg.linear.x = v
        msg.angular.z = w
        # Publish the data using a ROS publisher
        self._pub.publish(msg)


if __name__ == "__main__":
    ### Initialize this ROS node.
    # The init_node command makes ROS aware of this process. "user_name" is the
    # default name of this node, and it is overwritten if this script is
    # launched by a ROS launch file.
    rospy.init_node("user_interface")

    listener = PositionSubscriber()
    publisher = CmdPublisher()

    while not rospy.is_shutdown():
        user_input = raw_input(
            " q\t quit\n r\t reset\n x,y\t set goal to (x, y)\nYour command: "
        )
        ### Quit the script
        if user_input == "q":
            sys.exit(0)
        ### Reset the simulator
        # This is an example of ROS service call.
        if user_input == "r":
            client = rospy.ServiceProxy("/reset", std_srvs.srv.Empty)
            client.call()
        ### Run a controller to reach a goal
        # This is an example of ROS publisher.
        else:
            strings = user_input.split(",")
            new_goal = [float(s) for s in strings]
            # Our "algorithm"
            controller = Controller(*new_goal)
            t0 = time.time()
            while not rospy.is_shutdown():
                # Compute the velocity command using our algorithm
                v, w = controller(*listener.data)
                # Send the command via ROS publisher
                publisher(v, w)
                rospy.sleep(0.01)
                current_pos = np.array(listener.data[:2])
                if np.linalg.norm(current_pos - controller.goal) < 0.2:
                    rospy.loginfo("Goal reached")
                    break
                if time.time() - t0 > 5:
                    rospy.logwarn("Timeout reaching the goal!")
                    break

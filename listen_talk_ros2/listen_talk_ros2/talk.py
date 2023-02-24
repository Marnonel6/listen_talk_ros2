"""
Can load prewritten .mp3 files and play them back

PUBLISHERS:
    + turtle1/cmd_vel (Twist) - Linear and angular velocity to move the turtle forward and change its heading towards the waypoint.

SUBSCRIBERS:
    + turtle1/pose (Pose) - The position, orientation and velocity of the current turtle. This is used in the control algorithm to move the turtle through the waypoints list.

SERVICES:
    + toggle (Empty) - Changes the state of the system from ANY state to STOPPED or from STOPPED to MOVING
    + load (Waypoints) - Load an arbritary amount of waypoints

PARAMETERS:
    + frequency (double) - Sets the frequency of the timer, thus controlling the timer_callback function speed.
    + tolerance (double) - This is the maximum relative distance error that the turtle must have to verify that a waypoint was reached and thus the turtle can move to the next waypoint. 


"""

from enum import Enum, auto
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
import os
import ament_index_python
import os


class Talk(Node):
    """
    """

    def __init__(self):
        super().__init__("talk")

        # self.state = State.RESET # Starts with RESET state

        self.declare_parameter("frequency", 100.0, ParameterDescriptor(description="The velocity of the turtle"))
        self.declare_parameter("talk_file", ament_index_python.get_package_share_directory(
            "listen_talk_ros2") + "/German_shepherd_barking.mp3")
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.talk_file = self.get_parameter("talk_file").get_parameter_value().string_value

        # Publishers, Subscribers, Services and Timer
        self.sub = self.create_subscription(String, "/speed", self.update_pose, 10)
        self.timer = self.create_timer(1.0/self.frequency, self.timer_callback)


    def update_pose(self,data):
        """
        Subscribtion topic: speed
        """
        print("got data\n")
        if data.data == "up":
            blabla = 'mpg123' + ' ' + self.talk_file
            os.system(blabla)


    def timer_callback(self):
        """ 
        Timer Callback
        """


def main(args=None):
    """ The main() function. """

    rclpy.init(args=args)
    talk = Talk()
    talk.timer_callback()
    rclpy.spin(talk)
    rclpy.shutdown()


if __name__ == '__main__':
    main()




#
# Copyright 2020-2022 Picovoice Inc.
#
# You may not use this file except in compliance with the license. A copy of the license is located in the "LICENSE"
# file accompanying this source.
#
# Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on
# an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.
#

"""
Resets turtlesim and then allows user to load an arbritary amount of waypoints. The waypoints are then plotted using the turtle and a control algorithm is 
initiated to pubslish the twist and move the turtle through all the waypoints and back to the start.


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
from os import ST_APPEND
import re
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute, SetPen
from time import sleep
from math import atan, pi, atan2
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String
from turtlesim.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor

import argparse
import os
import sys
import struct
import wave
from threading import Thread

from picovoice import *
from pvrecorder import PvRecorder

# Cannot work offline needs to be connected to the internet to convert text to speach, but
# Can load prewritten .mp3 files when not connected to the internet just use the command below:
# os.system('mpg123 talk.mp3')

import os
from gtts import gTTS

# myText = 'Allan! Allan! Allan! No it is Steve! Oh! Steve! Steve! Steve!'
# myText = 'the roflcopter goes schwaschwaschwaschwahschawh' 
# myText = 'Jarvis.'
# myText = 'Hi my name is Jarvis and I am here to help.'
# myText = 'Whoof whoof whoof whoof'

# myOutput = gTTS(text=myText, lang='en', slow=False) # language - en, fr, es, it, de, in, uk
# myOutput.save('talk.mp3')

# os.system('mpg123 talk.mp3') # Or if in python directory
# Can load prewritten .mp3 files when not connected to the internet
# os.system('mpg123 /home/marno/Classes/Winter23/Winter_Project/listen_talk_ros/let_it_talk/talk.mp3')



class Talk(Node):
    """
    """

    def __init__(self):
        super().__init__("talk")

        # self.state = State.RESET # Starts with RESET state
        # self.flag_state_stopped = 0 # This is used to log "STOPPING" only once to debug.

        self.declare_parameter("frequency", 100.0, ParameterDescriptor(description="The velocity of the turtle"))
        self.declare_parameter("talk_file", "os.system('mpg123 /home/marno/Classes/Winter23/Winter_Project/listen_talk_ros/let_it_talk/talk.mp3')", ParameterDescriptor(description="The error tolerance for the waypoint"))
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.talk_file = self.get_parameter("talk_file").get_parameter_value().string_value
        # self.speed = String()

        # print(self.keyword_path)

        # Publishers, Subscribers, Services and Timer
        # self.pub = self.create_publisher(String, "/speed", 10)
        self.sub = self.create_subscription(String, "/speed", self.update_pose, 10)
        self.timer = self.create_timer(1.0/self.frequency, self.timer_callback)


    # def _wake_word_callback(self):
    #     print('[wake word]\n')


    def update_pose(self,data):
        """
        Subscribtion topic: turtle1/cmd_vel 
        """
        print("got data\n")
        if data.data == "up":
            print(data.data)
            os.system('mpg123 /home/marno/Classes/Winter23/Winter_Project/listen_talk_ros/let_it_talk/talk.mp3')


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




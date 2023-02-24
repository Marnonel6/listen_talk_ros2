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

import ament_index_python

class PicovoiceDemo(Thread):
    def __init__(
            self,
            access_key,
            audio_device_index,
            keyword_path,
            context_path,
            porcupine_library_path=None,
            porcupine_model_path=None,
            porcupine_sensitivity=0.5,
            rhino_library_path=None,
            rhino_model_path=None,
            rhino_sensitivity=0.5,
            endpoint_duration_sec=1.,
            require_endpoint=True,
            output_path=None):
        super(PicovoiceDemo, self).__init__()


class Listen(Node):
    """
    """

    def __init__(self):
        super().__init__("listen")

        self.cbgroup = ReentrantCallbackGroup()

        # self.state = State.RESET # Starts with RESET state
        self.flag_state_stopped = 0 # This is used to log "STOPPING" only once to debug.

        self.declare_parameter("frequency", 100.0, ParameterDescriptor(description="The velocity of the turtle"))
        # self.declare_parameter("keyword_path", "/home/marno/Classes/Winter23/Winter_Project/listen_talk_ros/let_it_talk/jarvis_linux.ppn", ParameterDescriptor(description="The error tolerance for the waypoint"))
        self.declare_parameter("keyword_path", ament_index_python.get_package_share_directory(
            "listen_talk_ros2")+ "/jarvis_linux.ppn")
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.keyword_path = self.get_parameter("keyword_path").get_parameter_value().string_value
        self.speed = String()

        # print(self.keyword_path)

        # Publishers, Subscribers, Services and Timer
        self.pub = self.create_publisher(String, "/speed", 10)
        # self.sub = self.create_subscription(Pose, "turtle1/pose", self.update_pose, 10)
        self.timer = self.create_timer(1.0/self.frequency, self.timer_callback, callback_group = self.cbgroup)


    def _wake_word_callback(self):
        print('[wake word]\n')

    def _inference_callback(self,inference):
        if inference.is_understood:
            print('{')
            print("  intent : '%s'" % inference.intent)
            print('  slots : {')
            for slot, value in inference.slots.items():
                print("    %s : '%s'" % (slot, value))
                self.speed.data = value # Publish Value value
            print('  }')
            print('}\n')
            
            # TODO PUBLISHES THE COMMAND
            self.pub.publish(self.speed)


            # os.system('mpg123 /home/marno/Classes/Winter23/Winter_Project/listen_talk_ros/let_it_talk/talk.mp3')

        else:
            print("Didn't understand the command.\n")

    def run(self):
        recorder = None
        wav_file = None
        # print("HALLLO")

        try:
            recorder = PvRecorder(device_index=-1, frame_length=self._picovoice.frame_length)
            recorder.start()

            # if self.output_path is not None:
            #     wav_file = wave.open(self.output_path, "w")
            #     # noinspection PyTypeChecker
            #     wav_file.setparams((1, 2, 16000, 512, "NONE", "NONE"))

            print("Using device: %s" % recorder.selected_device)
            print('[Listening ...]')

            while True:
                pcm = recorder.read()

                if wav_file is not None:
                    wav_file.writeframes(struct.pack("h" * len(pcm), *pcm))

                self._picovoice.process(pcm)
        except KeyboardInterrupt:
            sys.stdout.write('\b' * 2)
            print('Stopping ...')
        finally:
            if recorder is not None:
                recorder.delete()

            if wav_file is not None:
                wav_file.close()

            self._picovoice.delete()



    def show_audio_devices(self, cls):
        devices = PvRecorder.get_audio_devices()

        for i in range(len(devices)):
            print('index: %d, device name: %s' % (i, devices[i]))




    # def update_pose(self,data):
    #     """
    #     Subscribtion topic: turtle1/cmd_vel 
    #     """
    #     self.pose = data


    def timer_callback(self):
        """ 
        Timer Callback
        """

        # print(f"{self.keyword_path}")

        try:
            self._picovoice = Picovoice(
                access_key="bz3cScyGLZpi/dcR5/xHDJJ/pCBdswpMGXHL2Djgik7Rn04q54tdYA==",
                # keyword_path="/home/marno/Classes/Winter23/Winter_Project/listen_talk_ros/let_it_talk/jarvis_linux.ppn",
                keyword_path=self.keyword_path,
                wake_word_callback=self._wake_word_callback,
                context_path="/home/marno/Classes/Winter23/Winter_Project/listen_talk_ros/let_it_talk/Dog-command_en_linux_v2_1_0.rhn",
                inference_callback=self._inference_callback,
                porcupine_library_path=None,
                porcupine_model_path=None,
                porcupine_sensitivity=0.5,
                rhino_library_path=None,
                rhino_model_path=None,
                rhino_sensitivity=0.5,
                endpoint_duration_sec=1.0,
                require_endpoint=True)
            
            self.run()
            
            PicovoiceDemo(
                access_key="bz3cScyGLZpi/dcR5/xHDJJ/pCBdswpMGXHL2Djgik7Rn04q54tdYA==",
                audio_device_index=-1,
                # keyword_path="/home/marno/Classes/Winter23/Winter_Project/listen_talk_ros/let_it_talk/jarvis_linux.ppn",
                keyword_path=self.keyword_path,
                context_path="/home/marno/Classes/Winter23/Winter_Project/listen_talk_ros/let_it_talk/Dog-command_en_linux_v2_1_0.rhn",
                porcupine_library_path=None,
                porcupine_model_path=None,
                porcupine_sensitivity=0.5,
                rhino_library_path=None,
                rhino_model_path=None,
                rhino_sensitivity=0.5,
                endpoint_duration_sec=1.0,
                require_endpoint=True,
                output_path=None).run()



        except PicovoiceInvalidArgumentError as e:
            # args = (
            #     access_key,
            #     keyword_path,
            #     self._wake_word_callback,
            #     context_path,
            #     self._inference_callback,
            #     porcupine_library_path,
            #     porcupine_model_path,
            #     porcupine_sensitivity,
            #     rhino_library_path,
            #     rhino_model_path,
            #     rhino_sensitivity,
            #     endpoint_duration_sec,
            #     require_endpoint
            # )
            print("One or more arguments provided to Picovoice is invalid: ")
            print("If all other arguments seem valid, ensure that is a valid AccessKey")
            raise e
        except PicovoiceActivationError as e:
            print("AccessKey activation error")
            raise e
        except PicovoiceActivationLimitError as e:
            print("AccessKey has reached it's temporary device limit")
            raise e
        except PicovoiceActivationRefusedError as e:
            print("AccessKey refused")
            raise e
        except PicovoiceActivationThrottledError as e:
            print("AccessKey has been throttled")
            raise e
        except PicovoiceError as e:
            print("Failed to initialize Picovoice")
            raise e

        # self.audio_device_index = audio_device_index
        self.output_path = None


        # def main():
        #     parser = argparse.ArgumentParser()

        #     parser.add_argument(
        #         '--access_key',
        #         help='AccessKey obtained from Picovoice Console (https://console.picovoice.ai/)',
        #         required=True)

        #     parser.add_argument('--keyword_path', help="Absolute path to a Porcupine keyword file.")

        #     parser.add_argument('--context_path', help="Absolute path to a Rhino context file.")

        #     parser.add_argument('--porcupine_library_path', help="Absolute path to Porcupine's dynamic library.", default=None)

        #     parser.add_argument('--porcupine_model_path', help="Absolute path to Porcupine's model file.", default=None)

        #     parser.add_argument(
        #         '--porcupine_sensitivity',
        #         help="Sensitivity for detecting wake word. Each value should be a number within [0, 1]. A higher sensitivity " +
        #             "results in fewer misses at the cost of increasing the false alarm rate.",
        #         type=float,
        #         default=0.5)

        #     parser.add_argument('--rhino_library_path', help="Absolute path to Rhino's dynamic library.", default=None)

        #     parser.add_argument('--rhino_model_path', help="Absolute path to Rhino's model file.", default=None)

        #     parser.add_argument(
        #         '--rhino_sensitivity',
        #         help="Inference sensitivity. It should be a number within [0, 1]. A higher sensitivity value results in fewer" +
        #             "misses at the cost of (potentially) increasing the erroneous inference rate.",
        #         type=float,
        #         default=0.5)

        #     parser.add_argument(
        #         '--endpoint_duration_sec',
        #         help="Endpoint duration in seconds. An endpoint is a chunk of silence at the end of an utterance that marks "
        #             "the end of spoken command. It should be a positive number within [0.5, 5]. A lower endpoint duration "
        #             "reduces delay and improves responsiveness. A higher endpoint duration assures Rhino doesn't return "
        #             "inference pre-emptively in case the user pauses before finishing the request.",
        #         type=float,
        #         default=1.)

        #     parser.add_argument(
        #         '--require_endpoint',
        #         help="If set to `True`, Rhino requires an endpoint (a chunk of silence) after the spoken command. If set to "
        #             "`False`, Rhino tries to detect silence, but if it cannot, it still will provide inference regardless. "
        #             "Set to `False` only if operating in an environment with overlapping speech (e.g. people talking in the "
        #             "background).",
        #         default='True',
        #         choices=['True', 'False'])

        #     parser.add_argument('--audio_device_index', help='index of input audio device', type=int, default=-1)

        #     parser.add_argument('--output_path', help='Absolute path to recorded audio for debugging.', default=None)

        #     parser.add_argument('--show_audio_devices', action='store_true')

        #     args = parser.parse_args()

        #     if args.require_endpoint.lower() == 'false':
        #         require_endpoint = False
        #     else:
        #         require_endpoint = True

        #     if args.show_audio_devices:
        #         PicovoiceDemo.show_audio_devices()
        #     else:
                # if not args.keyword_path:
                #     raise ValueError("Missing path to Porcupine's keyword file.")

                # if not args.context_path:
                #     raise ValueError("Missing path to Rhino's context file.")

def main(args=None):
    """ The main() function. """

    rclpy.init(args=args)
    listen = Listen()
    listen.timer_callback()
    rclpy.spin(listen)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
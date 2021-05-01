#!/usr/bin/env python3

# Common Imports
import rospy, rospkg, roslib

from harmoni_common_lib.constants import State, ActuatorNameSpace
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf


# Specific Imports
from audio_common_msgs.msg import AudioData
import numpy as np
import base64 
import requests
from requests.exceptions import Timeout

# import wget
import contextlib
import ast
import wave
import os


class SpeakerService(HarmoniServiceManager):
    """Takes sound and publishes it to the default audio topic for the audio_play package

    Args:
        HarmoniServiceManager ([type]): [description]
    """

    def __init__(self, name, robot_ip):
        """ Initialization of variables and camera parameters """
        super().__init__(name)
        self.audio_publisher = rospy.Publisher(
            "/audio/audio",
            AudioData,
            queue_size=1,
        )
        self.state = State.INIT
        self.rospack = rospkg.RosPack()
        self.robot_ip = robot_ip
        return

    def request(self, data):
        """Publishes audio to the "/audio/audio" topic for the audio_play module

        Converts input audio from bytes or a local/network path to an audio msg.

        Args:
            data (str): This could be a string of:
                            - audio data
                            - path of local wav file
                            - link of wav audio file you want to download and heard from

        Returns:
            object: It containes information about the response received (bool) and response message (str)
                response: the state of the request
        """
        duration = 0
        print(data)
        self.state = State.REQUEST
        self.actuation_completed = False
        try:
            #data="/root/harmoni_catkin_ws/src/HARMONI/harmoni_actuators/harmoni_tts/temp_data/tts.wav"
            if type(data) == str:
                if ".wav" in data:
                    data = self.file_path_to_audio_data(data)
                    duration = data["duration"]
                    data = data["audio_data"]
                else:
                    data = ast.literal_eval(data)
                    data = base64.b64encode(data["audio_data"]) #test if this works
                    data = data.decode('utf-8')
            rospy.loginfo("Writing data for speaker")
            rospy.loginfo(f"length of data is {len(data)}")
            print(data)
            #Maximum size for the file is 3 Mb
            payload = {'fileName':'test.wav', 
                        "data": data,
                        "ImmediatelyApply": True, 
    #ImmediatelyApply flag make the robot reproduce the file
                        "OverwriteExisting": True
    #Always overwrite the same file, so it's discarded the next time something is reproduced
            }
            print("Sending request")
            print('http://{}/api/audio'.format(self.robot_ip))
            response = requests.post('http://{}/api/audio'.format(self.robot_ip), 
                                            params = payload,
                                            timeout = 1)
            print("receiving response")
            rospy.sleep(duration) #is it necessary? 
            self.state = State.SUCCESS
            self.response_received = True
            self.result_msg = response.text
            rospy.loginfo("Request successfully completed")
        except Timeout:
            rospy.logwarn("Speaker failed: The ip of the robot appears unreachable")
            self.state = State.FAILED
            self.response_received = True
            self.result_msg = ""
        except IOError as e: #can this really happen? i don't think so
            rospy.logwarn("Speaker failed: Audio appears too busy")
            self.state = State.FAILED
            self.response_received = True
            self.result_msg = ""
        return {"response": self.state}

    def file_path_to_audio_data(self, path):
        """Returns audio data from a local path or internet link
        TODO: Add wget to docker image

        Args:
            path (string): string of:
                            - local folder path
                            - link of audio file you want to listen to

        Returns:
            json: return an object with two fields:
                        - audio_data: base64 encoded file
                        - duration: int (duration of the file)
        """
        file_handle = path
        if "http" in path:
            url = path
            print("Beginning file download with wget module")
            file_handle = (
                self.rospack.get_path("harmoni_speaker") + "/temp_data/test.wav"
            )
        #it could be possible also to pass directly wav file 
        with open(file_handle, 'rb') as f:
            text = f.read()
            data = base64.b64encode(text)
            data = data.decode('utf-8')
        with contextlib.closing(wave.open(file_handle, "r")) as f:
            frames = f.getnframes()
            rate = f.getframerate()
            duration = frames / float(rate)
            rospy.loginfo(f"The audio lasts {duration} seconds")
        if "http" in path:
            os.remove(file_handle)
        return {"audio_data": data, "duration": duration}


def main():
    """Set names, collect params, and give service to server"""

    service_name = ActuatorNameSpace.speaker.name
    instance_id = rospy.get_param("/instance_id")
    service_id = f"{service_name}_{instance_id}"

    try:
        rospy.init_node(service_name)

        param = rospy.get_param("/robot_ip")
        rospy.loginfo(param)

        s = SpeakerService(service_id, param)

        service_server = HarmoniServiceServer(service_id, s)

        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

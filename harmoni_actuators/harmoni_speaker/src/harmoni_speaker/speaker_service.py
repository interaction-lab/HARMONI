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

    def do(self, data):
        """Publishes audio to the "/audio/audio" topic for the audio_play module

        Converts input audio from bytes or a local/network path to an audio msg.

        Args:
            data (str): This could be a string of:
                            - audio data
                            - path of local wav file
                            - link of wav audio file you want to download and heard from
        """
        duration = 0
        self.state = State.REQUEST
        self.actuation_completed = False
        try:
            if type(data) == str:
                if ".wav" in data:
                    data = self.file_path_to_audio_data(data)
                    duration = data["duration"]
                else:
                    data = ast.literal_eval(data)
            data = data["audio_data"]
            rospy.loginfo("Writing data for speaker")
            rospy.loginfo(f"length of data is {len(data)}")
            self.audio_publisher.publish(data)#.tostring() TODO test if tostring is necessary
            print("audiopub ok\n")
            #Maximum size for the file is 3 Mb
            response = requests.post('https://{}/api/audio'.format(self.robot_ip), 
                                            data = {'filename':'test.wav', 
                                                    "data":data,
                                                    "ImmediatelyApply": True, 
    #ImmediatelyApply flag make the robot reproduce the file
                                                    "OverwriteExisting": True},
    #Always overwrite the same file, so it's discarded the next time something is reproduced
                                                    timeout = 1) 
    #Timeout is set to one second. TODO test if it's enough
            print("apicall ok\n")
            rospy.sleep(duration)
            self.state = State.SUCCESS
            self.actuation_completed = True
        except Timeout:
            rospy.logwarn("Speaker failed: The ip of the robot appears unreachable")
            self.state = State.FAILED
            self.actuation_completed = True
        except IOError as e:
            rospy.logwarn("Speaker failed: Audio appears too busy")
            self.state = State.FAILED
            self.actuation_completed = True
            print(e)
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
            # wget.download(url, file_handle)
        #data = np.fromfile(file_handle, np.uint8)[24:]  # Loading wav file
        #it could be possible also to pass directly wav file 
        with open(file_handle, 'rb') as f:
            text = f.read()
            data = base64.b64encode(text)
        #data = data.astype(np.uint8).tostring()

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

        # params = rospy.get_param(service_name + "/" + instance_id + "_param/")
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

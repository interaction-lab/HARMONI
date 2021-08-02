#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import DetectorNameSpace, SensorNameSpace
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
from subprocess import Popen, PIPE
import numpy as np
import select
import pty
import os
import time
import re


class SpeechToTextService(HarmoniServiceManager):
    """
    Speech to text service using wave2letter
    """

    def __init__(self, name, param):
        """ Initialization of variables and w2l parameters """
        super().__init__(name)
        for key in param:
            setattr(self, key, param[key])
        rospy.loginfo("Setting ros params as class attributes")
        if not os.path.isdir(self.model_path):
            raise Exception(
                "W2L model has not been dowloaded", "Try running get_w2l_models.sh"
            )
        self.service_id = hf.get_child_id(self.name)
        self.w2l_process = None

        """Setup publishers and subscribers"""
        rospy.Subscriber("/audio/audio", AudioData, self.playing_sound_pause_callback)
        rospy.Subscriber(
            SensorNameSpace.microphone.value + self.subscriber_id,
            AudioData,
            self.sound_data_callback,
        )
        self.text_pub = rospy.Publisher(
            DetectorNameSpace.stt.value + self.service_id, String, queue_size=10
        )

        self.state = State.INIT
        return

    def start(self, rate=""):
        """Start the w2l stream and publish text"""
        # Startup stream if not started
        rospy.loginfo("Start the %s service" % self.name)
        if self.state == State.INIT or self.state == State.FAILED:
            self.transcribe_stream()  # Start the microphone service at the INIT
        else:
            self.state = State.START
        return

    def stop(self):
        """Stop the service"""
        rospy.loginfo("Stop the %s service" % self.name)
        try:
            self.state = State.SUCCESS
        except Exception:
            self.state = State.FAILED
        return

    def pause(self):
        """Set the service to success to stop publishing"""
        rospy.loginfo("Pause the %s service" % self.name)
        self.state = State.SUCCESS
        return

    def sound_data_callback(self, data):
        """Sends recieved data to w2l process"""
        # rospy.loginfo("The state is %s" % self.state)
        if self.state == State.START:
            if not self.w2l_process:
                rospy.loginfo("Callback occured before setup")
            else:
                # rospy.logdebug(f"W2L receiving data of size {len(data.data)}")
                self.w2l_process.stdin.write(data.data)
                self.w2l_process.stdin.flush()
        else:
            rospy.loginfo("Not Transcribing Audio, The state is %s" % self.state)
        return

    def playing_sound_pause_callback(self, data):
        """Sleeps when data is being published to the speaker"""
        rospy.loginfo(f"pausing for data: {len(data.data)}")
        self.pause()
        rospy.sleep(int(len(data.data) / 22040))
        self.start()
        return

    def transcribe_stream(self):
        """Setup W2L Process and read results as available"""
        r = rospy.Rate(20)

        rospy.loginfo("Opening up W2L process")
        self.w2l_process = Popen(
            ["{} --input_files_base_path={}".format(self.w2l_bin, self.model_path)],
            stdin=PIPE,
            stdout=PIPE,
            stderr=PIPE,
            shell=True,
        )

        rospy.loginfo("Setting up transcription")
        # Read setup text from w2l which is not needed
        for i in range(17):
            output = self.w2l_process.stdout.readline()
            # rospy.logdebug(output)

        # p = mp.Process(target=self.write_to_w2l, args=(,))
        # p.start()

        total_text = ""
        rospy.loginfo("Setup complete")
        self.state = State.START
        while not rospy.is_shutdown():
            output = self.w2l_process.stdout.readline()
            # rospy.logdebug(f" W2L ouptut: {output}")
            text = fix_w2l_text(output)
            if text:
                total_text = total_text + " " + text
            else:
                if total_text:
                    rospy.loginfo("Heard:" + total_text)
                    # Publish without the first space
                    self.text_pub.publish(total_text[1:])
                    total_text = ""
            r.sleep()
        rospy.logdebug("No longer transcribing the stream")


def fix_w2l_text(output):
    """W2L text has some predictabile peculiarities, this function strips those out

    Args:
        output (string): Raw output of W2L inference

    Returns:
        str: clean tet of what was said
    """
    text = output.decode("utf-8")
    if len(text) > 1:
        text = text.split(",")[2][:-2]
    else:
        return
    # Remove some bad outputs
    if len(text) > 0:
        if text[0] == "h" and len(text) == 1:
            text = ""
    if len(text) > 1:
        if text[:2] == "h " or text == " transcriptio":
            text = ""
    return text


def main():
    """Set names, collect params, and give service to server"""

    service_name = DetectorNameSpace.stt.name  # "w2l"
    instance_id = rospy.get_param("instance_id")  # "default"
    service_id = f"{service_name}_{instance_id}"

    try:
        rospy.init_node(service_name, log_level=rospy.DEBUG)

        # w2l/default_param/[all your params]
        params = rospy.get_param(service_name + "/" + instance_id + "_param/")

        s = SpeechToTextService(service_id, params)

        service_server = HarmoniServiceServer(service_id, s)

        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

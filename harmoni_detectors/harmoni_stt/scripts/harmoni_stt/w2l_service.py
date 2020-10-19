#!/usr/bin/env python

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
        super().__init__(name)
        """ Initialization of variables and w2l parameters """
        self.subscriber_id = param["subscriber_id"]
        self.model_path = param["model_path"]
        if not os.path.isdir(self.model_path):
            raise Exception(
                "W2L model has not been dowloaded", "Try running get_w2l_models.sh"
            )
        self.w2l_bin = param["w2l_bin"]
        self.service_id = hf.get_child_id(self.name)
        self.w2l_process = None
        """Setup publishers and subscribers"""
        rospy.Subscriber(
            SensorNameSpace.microphone.value + self.subscriber_id,
            AudioData,
            self.callback,
        )
        rospy.Subscriber("/audio/audio", AudioData, self.pause_back)
        self.text_pub = rospy.Publisher(
            DetectorNameSpace.stt.value + self.service_id, String, queue_size=10
        )
        """Setup the stt service as server """
        self.state = State.INIT
        return

    def pause_back(self, data):
        rospy.loginfo(f"pausing for data: {len(data.data)}")
        self.pause()
        rospy.sleep(int(len(data.data) / 30000))  # TODO calibrate this guess
        self.state = State.START

    def start(self, rate=""):
        rospy.loginfo("Start the %s service" % self.name)
        if self.state == State.INIT:
            self.state = State.START
            self.transcribe_stream()  # Start the microphone service at the INIT
        else:
            self.state = State.START
        return

    def stop(self):
        rospy.loginfo("Stop the %s service" % self.name)
        try:
            self.close_stream()
            self.state = State.SUCCESS
        except Exception:
            self.state = State.FAILED
        return

    def pause(self):
        rospy.loginfo("Pause the %s service" % self.name)
        self.state = State.SUCCESS
        return

    def callback(self, data):
        # rospy.loginfo("The state is %s" % self.state)
        if self.state == State.START:
            if not self.w2l_process:
                rospy.loginfo("Callback occured before setup")
            else:
                self.w2l_process.stdin.write(data.data)
                self.w2l_process.stdin.flush()
        else:
            rospy.loginfo("Not Transcribing Audio, The state is %s" % self.state)
        return

    def transcribe_stream(self):
        rospy.loginfo("Openning up W2L process")
        self.w2l_process = Popen(
            ["{} --input_files_base_path={}".format(self.w2l_bin, self.model_path)],
            stdin=PIPE,
            stdout=PIPE,
            stderr=PIPE,
            shell=True,
        )
        """Listening from the microphone """
        rospy.loginfo("Setting up transcription")
        for i in range(17):
            output = self.w2l_process.stdout.readline()
        # p = mp.Process(target=self.write_to_w2l, args=(,))
        # p.start()
        total_text = ""
        rospy.loginfo("Setup complete")
        while not rospy.is_shutdown():
            output = self.w2l_process.stdout.readline()
            text = self.fix_text(output)
            if text:
                total_text = total_text + " " + text
            else:
                if total_text:
                    rospy.loginfo("Heard:" + total_text)
                    self.text_pub.publish(total_text[1:])
                    total_text = ""

    def fix_text(self, output):
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
    service_name = DetectorNameSpace.stt.name
    name = rospy.get_param("/name_" + service_name + "/")
    test = rospy.get_param("/test_" + service_name + "/")
    test_input = rospy.get_param("/test_input_" + service_name + "/")
    test_id = rospy.get_param("/test_id_" + service_name + "/")
    try:
        rospy.init_node(service_name)
        param = rospy.get_param(name + "/" + test_id + "_param/")
        if not hf.check_if_id_exist(service_name, test_id):
            rospy.logerr(
                "ERROR: Remember to add your configuration ID also in the harmoni_core config file"
            )
            return
        service = hf.set_service_server(service_name, test_id)
        s = SpeechToTextService(service, param)
        service_server = HarmoniServiceServer(name=service, service_manager=s)
        if test:
            rospy.loginfo("Testing the %s" % (service))
            s.start()
        else:
            service_server.update_feedback()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

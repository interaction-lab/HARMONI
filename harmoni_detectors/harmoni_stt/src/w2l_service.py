#!/usr/bin/env python3.6
# NEW
# Importing the libraries
from subprocess import Popen, PIPE
import select
import pty
import os
import time
import re
import numpy as np
import rospy
from harmoni_common_lib.constants import State, RouterDetector, RouterSensor
from harmoni_common_lib.helper_functions import HelperFunctions
from harmoni_common_lib.child import InternalServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String


class SpeechToTextService(HarmoniServiceManager):
    """
    Speech to text service using wave2letter
    """

    def __init__(self, name, param):
        """ Initialization of variables and w2l parameters """
        rospy.loginfo("Wav2Letter initializing")
        self.name = name
        self.subscriber_id = param["subscriber_id"]
        self.model_path = param["model_path"]
        if not os.path.isdir(self.model_path):
            raise Exception(
                "W2L model has not been dowloaded", "Try running get_w2l_models.sh"
            )
        self.w2l_bin = param["w2l_bin"]
        self.service_id = HelperFunctions.get_child_id(self.name)
        """Setup publishers and subscribers"""
        rospy.Subscriber(
            RouterSensor.microphone.value + self.subscriber_id,
            AudioData,
            self.callback,
        )
        self.text_pub = rospy.Publisher(
            RouterDetector.stt.value + self.service_id, String, queue_size=10
        )
        """Setup the stt service as server """
        self.state = State.INIT
        super().__init__(self.state)
        return

    def state_update(self):
        super().update(self.state)
        return

    def test(self):
        super().test()
        rospy.loginfo("Test the %s service" % self.name)
        success = True
        return success

    def start(self, rate=""):
        rospy.loginfo("Start the %s service" % self.name)
        super().start(rate)
        if self.state == State.INIT:
            self.state = State.START
            self.state_update()
            self.transcribe_stream()  # Start the microphone service at the INIT
        else:
            self.state = State.START
        # self.state_update()
        return

    def stop(self):
        rospy.loginfo("Stop the %s service" % self.name)
        super().stop()
        try:
            self.close_stream()
            self.state = State.SUCCESS
            self.state_update()
        except:
            self.state = State.FAILED
            self.state_update()
        return

    def pause(self):
        rospy.loginfo("Pause the %s service" % self.name)
        super().pause()
        self.state = State.SUCCESS
        self.state_update()
        return

    def callback(self, data):
        # rospy.loginfo("The state is %s" % self.state)
        if self.state == State.START:

            # self.w2l_process.stdin.write(np.asarray(data.data, dtype=np.byte))
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
        text = text.split(",")[2][:-2]
        # Remove some bad outputs
        if len(text) > 0:
            if text[0] == "h" and len(text) == 1:
                text = ""
        if len(text) > 1:
            if text[:2] == "h " or text == " transcriptio":
                text = ""
        return text


def main():
    service_name = RouterDetector.stt.name
    name = rospy.get_param("/name_" + service_name + "/")
    test = rospy.get_param("/test_" + service_name + "/")
    input_test = rospy.get_param("/input_test_" + service_name + "/")
    id_test = rospy.get_param("/id_test_" + service_name + "/")
    try:
        rospy.init_node(service_name)
        list_service_names = HelperFunctions.get_child_list(service_name)
        service_server_list = []
        last_event = ""
        for service in list_service_names:
            print(service)
            service_id = HelperFunctions.get_child_id(service)
            param = rospy.get_param(name + "/" + service_id + "_param/")
            s = SpeechToTextService(service, param)
            service_server_list.append(
                InternalServiceServer(name=service, service_manager=s)
            )
            if test and (service_id == id_test):
                rospy.loginfo("Testing the %s" % (service))
                s.start()
                s.transcribe_file(input_test)
        if not test:
            for server in service_server_list:
                server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

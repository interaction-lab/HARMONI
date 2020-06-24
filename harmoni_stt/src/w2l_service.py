#!/usr/bin/env python3.6

# Importing the libraries
from subprocess import Popen, PIPE
import select
import pty
import os
import time
import re
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
            raise Exception("W2L model has not been dowloaded", "Try running get_w2l_models.sh")
        self.w2l_bin = param["w2l_bin"]
        self.service_id = HelperFunctions.get_child_id(self.name)
        """Setup publishers and subscribers"""
        rospy.Subscriber(RouterSensor.microphone.value + self.subscriber_id+ "/talking"  , AudioData, self.callback)
        self.text_pub = rospy.Publisher(RouterDetector.stt.value + self.service_id , String, queue_size=10)
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
        self.set_w2l_proc()
        super().start(rate)
        if self.state == State.INIT:
            self.state = State.START
            self.state_update()
        else:
            self.state = State.START
        self.state_update()
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

    def set_w2l_proc(self):
        self.w2l_process = Popen(['{} --input_files_base_path={}'.format(self.w2l_bin, self.model_path)],
                                 bufsize=1,
                                 stdin=PIPE, stdout=PIPE, stderr=PIPE,
                                 shell=True,
                                 close_fds=True)
        return

    def callback(self, data):
        if self.state == State.START:
            rospy.loginfo("Transcribing")
            text = self.transcribe_bytes(data.data)
            if text:
                self.text_pub.publish(text)
        else:
            rospy.loginfo("Not Transcribing Audio")
        return

    def transcribe_file(self, file_name):
        """ Transcription of audio into text from file"""
        rospy.loginfo("Transcription of audio file")
        with open(file_name, mode='rb') as wav_file:
            wav_contents = wav_file.read()
        self.transcribe_bytes(wav_contents)
        return

    def transcribe_bytes(self, b_string):
        """ Transcription of bytes"""
        rospy.loginfo("Transcription of the bytes")
        outs, errs = self.w2l_process.communicate(input=b_string, timeout=15)
        print(outs, errs)
        text_list = self.fix_text(outs)
        rospy.loginfo("The text list is %s" %text_list)
        if not any(text_list):
            self.set_w2l_proc()
            return
        self.set_w2l_proc()
        text_list = [t for t in text_list if t]
        return ' '.join(text_list)

    def fix_text(self, text):
        output_by_sec = ' '.join(re.split(r'[,\s]', text.decode("utf-8"))[95:-13]).split('  ')
        output_by_sec = [' '.join(sec.split(' ')[2:]) for sec in output_by_sec]
        final_output = []
        for sec in output_by_sec:  # Exclude some bad outputs
            if len(sec) > 0:
                if (sec[0] == 'h' and len(sec) == 1):
                    sec = ''
            if len(sec) > 1:
                if sec[:2] == 'h ':
                    sec = ''
            final_output.append(sec)
        return(final_output)

def main():
    test = rospy.get_param("/test/")
    input_test = rospy.get_param("/input_test/")
    id_test = rospy.get_param("/id_test/")
    try:
        service_name = RouterDetector.stt.name
        rospy.init_node(service_name)
        list_service_names = HelperFunctions.get_child_list(service_name)
        service_server_list = []
        last_event = "" 
        for service in list_service_names:
            print(service)
            service_id = HelperFunctions.get_child_id(service)
            param = rospy.get_param("~"+service_id+"_param/")
            s = SpeechToTextService(service, param)
            service_server_list.append(InternalServiceServer(name=service, service_manager=s))
            if test and (service_id == id_test):
                rospy.loginfo("Testing the %s" %(service))
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

#!/usr/bin/env python3.6

# Importing the libraries
from subprocess import Popen, PIPE
import select
import pty
import os
import sys
import time
import re

import rospy
from audio_common_msgs.msg import AudioData


class SpeechToText(object):
    def __init__(self):
        self.model_path = "/root/model/"
        self.w2l_bin = "/root/wav2letter/build/inference/inference/examples/simple_streaming_asr_example"
        rospy.Subscriber('/harmoni/sensing/listening/microphone', AudioData, self.callback)
        self.set_w2l_proc()
        self.sound = []
        print('init complete')

    def test(self):
        file_name = '/root/audio/test4.wav'
        print(file_name)
        self.transcribe_file(file_name)

    def set_w2l_proc(self):
        self.w2l_process = Popen(['{} --input_files_base_path={}'.format(self.w2l_bin, self.model_path)],
                                 bufsize=1,
                                 stdin=PIPE, stdout=PIPE, stderr=PIPE,
                                 shell=True,
                                 close_fds=True)
        return

    def callback(self, data):
        #print(data.data, type(data.data))
        # hexdata = ''.join([chr(item) for item in data.data])
        #print('audio recieved')
        self.transcribe_bytes(data.data)
        return

    def transcribe_file(self, file_name):
        with open(file_name, mode='rb') as wav_file:
            wav_contents = wav_file.read()
        self.transcribe_bytes(wav_contents)

    def transcribe_bytes(self, b_string):
        outs, errs = self.w2l_process.communicate(input=b_string, timeout=15)
        #print(outs)
        text_list = self.fix_text(outs)
        if not any(text_list):
            self.set_w2l_proc()
            return
        print(text_list)
        print(' '.join(text_list))
        self.set_w2l_proc()

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
    try:
        rospy.init_node('stt')
        stt = SpeechToText()
        # stt.test()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
import pyaudio
import math
import audioop
import numpy as np
from collections import deque
# from harmoni_common_lib.child import HarwareReadingServer
# from harmoni_common_lib.service_manager import HarmoniServiceManager
from audio_common_msgs.msg import AudioData

class Status():
    """ Status of the microphone service """
    INIT = 0 # init the service
    LISTENING = 1 # start listen to the voice
    NOT_LISTENING = 2 # stop listen to the voice
    END = 3  # terminate the service


class MicrophoneService():
    """
    Microphone service
    """

    def __init__(self, name, param):
        """ Initialization of variables and microphone parameters """
        rospy.loginfo("MicrophoneService initializing")
        rospy.loginfo("params:")
        rospy.loginfo(param)
        self.name = name
        self.audio_format_width = param["audio_format_width"]
        self.chunk_size = param["chunk_size"]
        self.total_channels = param["total_channels"]
        self.audio_rate = param["audio_rate"]
        self.silence_limit_seconds = param["silence_limit_seconds"]
        self.previous_audio_seconds = param["previous_audio_seconds"]
        self.total_silence_samples = param["total_silence_samples"]
        self.silence_threshold = param["silence_threshold"]
        self.loudest_sound_value = param["loudest_sound_value"]
        self.device_name = param["device_name"]
        self.set_threshold = param["set_threshold"]
        """ Setup the microphone """
        self.p = pyaudio.PyAudio()
        self.audio_format = pyaudio.paInt16  # How can we trasform it in a input parameter?
        self.stream = None
        self.setup_microphone()
        """Init the publisher """
        self.mic_pub = rospy.Publisher("/harmoni/sensing/listening/microphone", AudioData, queue_size=1) # Publishing the voice data
        self.mic_raw_pub = rospy.Publisher("/harmoni/sensing/microphone", AudioData, queue_size=1) # Publishing raw_data
        """Setup the microphone service as server """
        self.status = Status.INIT 
        # #super(MicrophoneService, self).__init__(self.status)
        return

    # def status_update(self):
    #     #super(MicrophoneService, self).update(self.status)
    #     return

    # def test(self):
    #     #super(MicrophoneService, self).test()
    #     rospy.loginfo("Test the %s service" % self.name)
    #     success = True
    #     return success

    def start(self):
        rospy.loginfo("Start the %s service" % self.name)
        # #super(MicrophoneService, self).start(rate)
        if self.status == 0:
            self.status = Status.LISTENING
            # self.status_update()
            try:
                self.open_stream()
                self.listen() # Start the microphone service at the INIT
            except:
                self.status = Status.END
        else:
            self.status = Status.LISTENING
        # self.status_update()
        #self.listen()

    def stop(self):
        rospy.loginfo("Stop the %s service" % self.name)
        #super(MicrophoneService, self).stop()
        self.close_stream()
        self.status = Status.END
        # self.status_update()

    def pause(self):
        rospy.loginfo("Pause the %s service" % self.name)
        #super(MicrophoneService, self).pause()
        self.status = Status.NOT_LISTENING
        # self.status_update()

    def setup_microphone(self):
        """ Setup the microphone """
        rospy.loginfo("Setting up the %s" % self.name)
        self.get_index_device()  # get index of the input audio device
        self.determine_silence_threshold(self.set_threshold)
        return

    def open_stream(self):
        """Opening the stream """
        rospy.loginfo("Opening the audio input stream")
        self.stream = self.p.open(
            format=self.audio_format,
            channels=self.total_channels,
            rate=self.audio_rate, input=True,
            input_device_index=self.input_device_index,
            frames_per_buffer=self.chunk_size
        )
        return

    def close_stream(self):
        """Closing the stream """
        self.stream.stop_stream()
        self.stream.close()
        return

    def listen(self):
        """Listening from the microphone """
        rospy.loginfo("The %s is listening" % self.name)
        current_audio = b""
        chunks_per_second = int(self.audio_rate / self.chunk_size)
        sliding_window = deque(maxlen=self.silence_limit_seconds * chunks_per_second)
        prev_audio = deque(maxlen=self.previous_audio_seconds * chunks_per_second)
        started = False
        while not rospy.is_shutdown():
            # rospy.loginfo("while:")
            if self.status != Status.END:
                # rospy.loginfo("Not end:")
                # Grab audio
                latest_audio_data = self.stream.read(self.chunk_size,exception_on_overflow=False)
                # rospy.loginfo("Stuck A?")
                raw_audio_bitstream = np.fromstring(latest_audio_data, np.uint8)
                # rospy.loginfo("Stuck B?")
                raw_audio = raw_audio_bitstream.tolist()
                # rospy.loginfo("Stuck C?")
                self.mic_raw_pub.publish(raw_audio) # Publishing raw AudioData
                # rospy.loginfo("Stuck D?")

                if self.status == Status.LISTENING:
                    # Check magnitude of audio
                    sliding_window.append(math.sqrt(abs(audioop.avg(latest_audio_data, self.audio_format_width))))
                    print_window = str([round(x,1) for x in sliding_window])
                    maximum = round(max(sliding_window), 2)
                    # rospy.loginfo("Noises" + print_window + str(maximum))

                    if any([x > self.silence_threshold for x in sliding_window]):
                        if not started:
                            rospy.loginfo("Sound detected")
                            started = True
                        # rospy.loginfo("Loud sound in buffer")
                        # print(latest_audio_data)
                        current_audio += latest_audio_data
                        # rospy.loginfo("Latest audio added to current")
                    elif started:
                        rospy.loginfo("Finished detecting")
                        all_audio_data = b"".join(prev_audio) + current_audio
                        self.status = Status.NOT_LISTENING
                        # print(all_audio_data)
                        audio_bitstream = np.fromstring(all_audio_data, np.uint8)
                        audio = audio_bitstream.tolist()
                        self.mic_pub.publish(audio)  # Publishing AudioData of voice
                        started = False
                        sliding_window.clear()
                        prev_audio.clear()
                        current_audio = b""
                        self.status = Status.LISTENING
                        rospy.loginfo("Detection sent. Listening")
                        # self.status_update()
                    else:
                        # rospy.loginfo("No loud sound detected")
                        prev_audio.append(latest_audio_data)
                        # rospy.loginfo("Recording for posterity")
                else:
                    rospy.loginfo("not listening")
            else:
                break
        rospy.loginfo("Shutting down")
        return

    def get_index_device(self):
        """ 
        Find the input audio devices configured in ~/.asoundrc. 
        If the device is not found, pyaudio will use your machine default device
        """
        for i in range(self.p.get_device_count()):
            device = self.p.get_device_info_by_index(i)
            if device["name"] == self.device_name:
                rospy.loginfo("Found device with name " + self.device_name)
                self.input_device_index = i
                return

    def determine_silence_threshold(self, mode):
        """Determine silence threshold from the mic or setting a constant value """
        loudest_sound_cohort_perc = 0.3
        silence_threshold_multiplier = 1.5
        if mode == "default":
            rospy.loginfo("Getting audio intensity")
            self.open_stream()

            values = [math.sqrt(abs(audioop.avg(
                                        self.stream.read(self.chunk_size), 
                                        self.audio_format_width))) \
                                for _ in range(self.total_silence_samples)]

            values = sorted(values, reverse=True) # Only take the loudest values in the silence 
            print(values)

            total_samples_in_cohort = int(self.total_silence_samples * loudest_sound_cohort_perc)

            sum_of_loudest_sounds = sum(values[:total_samples_in_cohort])

            average_of_loudest_sounds = sum_of_loudest_sounds / total_samples_in_cohort
            self.close_stream()
        elif mode == "constant":
            rospy.loginfo("Using constant audio intensity")
            average_of_loudest_sounds = self.loudest_sound_value

        rospy.loginfo("Average audio intensity is " + str(average_of_loudest_sounds))
        self.silence_threshold = average_of_loudest_sounds * silence_threshold_multiplier
        rospy.loginfo("Silence threshold set to " + str(self.silence_threshold))
        return


def main():
    try:
        service_name = "microphone"
        rospy.init_node(service_name + "_node")
        param = rospy.get_param("/"+service_name+"_param/")
        s = MicrophoneService(service_name, param)
        s.open_stream()
        s.start()
        # s.listen()
        # hardware_reading_server = HarwareReadingServer(name=service_name, service_manager=s)
        # hardware_reading_server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

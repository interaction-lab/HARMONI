#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import State, DetectorNameSpace, SensorNameSpace
from audio_common_msgs.msg import AudioData
from google.cloud import speech
from std_msgs.msg import String
import numpy as np
import os
import io
import time
from six.moves import queue

class STTGoogleService(HarmoniServiceManager):
    """
    Google service
    """

    def __init__(self, name, param):
        super().__init__(name)
        """ Initialization of variables and google parameters """
        self.sample_rate = param["sample_rate"]
        self.language = param["language_id"]
        self.audio_channel = param["audio_channel"]
        self.credential_path = param["credential_path"]
        self.subscriber_id = param["subscriber_id"]
        self.max_duration = param["max_duration"]
        self.start_time = None
        self.elapsed_time = None
        self.service_id = hf.get_child_id(self.name)
        self.result_msg = ""
        self.stt_response = ""

        self._buff = queue.Queue()
        self.closed = False

        """ Setup the google request """
        self.setup_google()

        """Setup the google service as server """
        self.response_text = ""
        self.data = b""

        """Setup publishers and subscribers"""
        rospy.Subscriber(
            SensorNameSpace.microphone.value + self.subscriber_id,
            AudioData,
            self.callback,
        )
        rospy.Subscriber("/audio/audio", AudioData, None)

        self.text_pub = rospy.Publisher(
            DetectorNameSpace.stt.value + self.service_id, String, queue_size=10
        )

        rospy.Subscriber(
            DetectorNameSpace.stt.value + self.service_id,
            String,
            self.stt_callback,
        )

        """Setup the stt service as server """
        self.state = State.INIT
        return

    def pause_back(self, data):
        rospy.loginfo(f"pausing for data: {len(data.data)}")
        self.pause()
        rospy.sleep(int(len(data.data) / 30000))  # TODO calibrate this guess
        self.state = State.START
        return

    def setup_google(self):
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = self.credential_path
        self.client = speech.SpeechClient()
        self.config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=self.sample_rate,
            language_code=self.language,
            max_alternatives=1,
        )
        self.streaming_config = speech.StreamingRecognitionConfig(
            config=self.config, interim_results=True
        )
        rospy.loginfo("SETUP FINISHED")
        return

    def callback(self, data):
        """ Callback function subscribing to the microphone topic"""
        #rospy.loginfo("Add data to buffer")
        if self.state == State.REQUEST:
            self._buff.put(data.data)
        # rospy.loginfo("Items in buffer: "+ str(self._buff.qsize()))

        # else:
            # rospy.loginfo("Not Transcribing data")

    def listen_print_loop(self,responses):
        """ Prints responses coming from Google STT """ 
        self.stt_response = ""

        for response in responses:
            if not response.results:
                continue

            rospy.loginfo(f"Response: {response}")

            # The `results` list is consecutive. For streaming, we only care about
            # the first result being considered, since once it's `is_final`, it
            # moves on to considering the next utterance.
            result = response.results[0]
            if not result.alternatives:
                continue

            # Display the transcription of the top alternative.
            transcript = result.alternatives[0].transcript
            # self.text_pub.publish(transcript)
            # self.response_received = True
            
            for result in response.results:
                if result.is_final:
                    # rospy.loginfo(result.alternatives[0].transcript)
                    rospy.loginfo("STT response text: "+ result.alternatives[0].transcript)
                    self.stt_response = result.alternatives[0].transcript
                    self.text_pub.publish(self.stt_response)
                    self.response_received = True
    
    def listen_print_untill_result_is_final(self,responses):
        """ Prints responses coming from Google STT """ 
        self.stt_response = ""

        self.start_time = time.time()

        for response in responses:
            if not response.results:
                continue
            #rospy.loginfo(f"Response: {response}")
            self.start_time = time.time()
            
            result = response.results[0]
            if not result.alternatives:
                continue

            #transcript = result.alternatives[0].transcript
            for result in response.results:
                if result.is_final:
                    # rospy.loginfo(result.alternatives[0].transcript)
                    rospy.loginfo("STT response text: "+ result.alternatives[0].transcript)
                    self.stt_response = result.alternatives[0].transcript
                    self.text_pub.publish(self.stt_response)
                    self.response_received = True
                    return

    def transcribe_file_request(self, data):
        """ Transcribes a single audio file """

        rate = ""  # TODO: TBD
        audio = {"content": data}
        try:
            rospy.loginfo("Request to google")
            operation = self.client.long_running_recognize(
                config=self.config, audio=audio
            )
            rospy.loginfo("Waiting for the operation to complete.")
            self.state = State.PAUSE
            response = operation.result()
            for result in response.results:
                self.data = b""
                alternative = result.alternatives[0]
                text = alternative.transcript
                rospy.loginfo("The response is %s" % (text))
                print(self.response_text)
                if text:
                    self.response_text = self.response_text + " " + text
                else:
                    if self.response_text:
                        rospy.loginfo("Heard:" + self.response_text)
                        self.text_pub.publish(self.response_text[1:])
                        self.response_text = ""
            self.state = State.START
        except rospy.ServiceException:
            self.start = State.FAILED
            rospy.loginfo("Service call failed")
            self.response_received = True
            self.result_msg = ""
        return

    def stt_callback(self, data):
        """ Callback function subscribing to the microphone topic"""
        self.response_received = True


    def request(self, data):
        rospy.loginfo("Start the %s request" % self.name)
        self.state = State.REQUEST
        self.stt_response = "null"
        self.response_received = False
        try:
            # Transcribes data coming from microphone 

            audio_generator = self.generator()
            self.requests = (
                speech.StreamingRecognizeRequest(audio_content=content)
                for content in audio_generator
            )

            self.start_time = time.time() #time.time() is the current time
            print("Timer started at: ", self.start_time)

            responses = self.client.streaming_recognize(self.streaming_config, self.requests)

            if self.max_duration < self.elapsed_time:
                print("Timeout!")
                print("STT response text: "+ self.stt_response)
                self.response_received = True
                self.state = State.SUCCESS
            else:
                self.listen_print_untill_result_is_final(responses)

            r = rospy.Rate(1)
            while not self.response_received:
                r.sleep()

            self.state = State.SUCCESS
            self.result_msg = self.stt_response
            self.response_received = True

        except rospy.ServiceException:
            self.start = State.FAILED
            rospy.loginfo("Service call failed")
            self.response_received = True
            self.result_msg = ""
        finally:
            self._buff.queue.clear()
        return {"response": self.state, "message": self.result_msg}

    def wav_to_data(self, path):
        with io.open(path, "rb") as f:
            content = f.read()
        return content

    def generator(self):
        """ Generator of data for Google STT """
        # From https://cloud.google.com/speech-to-text/docs/streaming-recognize
        
        while not self.closed:
            self.elapsed_time = time.time() - self.start_time
            print("elapsed: ",self.elapsed_time)

            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            
            if chunk is None:
                print("chunk is None")
                return
            data = [chunk]

            if self.max_duration < self.elapsed_time:
                print("end")
                return

            #TODO prova senza questo while
            # Now consume whatever other data's still buffered.
            
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        print("chunk is None")
                        return
                    data.append(chunk)
                except queue.Empty:  
                    break

            yield b"".join(data)


    def start(self, rate=""):
        try:
            rospy.loginfo("Start the %s service" % self.name)
            if self.state == State.INIT:
                self.state = State.START
                
                # Transcribes data coming from microphone 
                """
                audio_generator = self.generator()
                self.requests = (
                    speech.StreamingRecognizeRequest(audio_content=content)
                    for content in audio_generator
                )
                responses = self.client.streaming_recognize(self.streaming_config, self.requests)
                self.listen_print_loop(responses)
                """

            else:
                self.state = State.START

        except Exception:
            rospy.loginfo("Killed the %s service" % self.name)
        return

    def stop(self):
        rospy.loginfo("Stop the %s service" % self.name)
        try:            
            # Signal the STT input data generator to terminate so that the client's
            # streaming_recognize method will not block the process termination.
            self.closed = True
            self._buff.put(None)

            self.state = State.SUCCESS
        except Exception:
            self.state = State.FAILED
        return

    def pause(self):
        rospy.loginfo("Pause the %s service" % self.name)
        self.state = State.SUCCESS
        return

def main():
    """Set names, collect params, and give service to server"""

    service_name = DetectorNameSpace.stt.name  # "stt"
    instance_id = rospy.get_param("instance_id")  # "default"
    service_id = f"{service_name}_{instance_id}"
    try:
        rospy.init_node(service_name, log_level=rospy.DEBUG)

        # stt/default_param/[all your params]
        params = rospy.get_param(service_name + "/" + instance_id + "_param/")

        s = STTGoogleService(service_id, params)

        service_server = HarmoniServiceServer(name=service_id, service_manager=s)

        print(service_name)
        print("**********************************************************************************************")
        print(service_id)

        #s.start()

        # Streaming audio from mic
        service_server.start_sending_feedback()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
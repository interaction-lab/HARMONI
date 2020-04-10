#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
import boto3
import re
import json
import soundfile as sf
import numpy as np
from botocore.exceptions import BotoCoreError, ClientError
from contextlib import closing
from harmoni_common_lib.child import WebServiceServer
from harmoni_common_lib.service_manager import HarmoniExternalServiceManager

class Status():
    """ Status of the tts service """
    INIT = 0 # init the service
    REQUEST_START = 1 # start the request
    RESPONSE_RECEIVED = 2 # receive the response
    REQUEST_FAILED = 3  # terminate the service


class AWSTtsService(HarmoniExternalServiceManager):
    """
    Amazon tts service
    """

    def __init__(self, name, param):
        """ Initialization of variables and tts parameters """
        rospy.loginfo("AWS Polly initializing")
        self.name = name
        self.region_name = param["region_name"]
        self.voice = param['voice']
        self.language = param["language"]
        self.outdir = param["outdir"]
        self.wav_header_length = param["wav_header_length"]
        """ Setup the tts request """
        self.setup_aws_tts()
        """Setup the tts service as server """
        self.status = Status.INIT 
        super(AWSTtsService, self).__init__(self.status)
        return

    def setup_aws_tts(self):
        self.tts = boto3.client('polly', region_name=self.region_name)
        self.vis_transl = {"p": "BILABIAL",
                    "f": "LABIODENTAL",
                    "T": "INTERDENTAL",
                    "s": "DENTAL_ALVEOLAR",
                    "t": "DENTAL_ALVEOLAR",
                    "S": "POSTALVEOLAR",
                    "r": "POSTALVEOLAR",
                    "k": "VELAR_GLOTTAL",
                    "i": "CLOSE_FRONT_VOWEL",
                    "u": "CLOSE_BACK_VOWEL",
                    "@": "MID_CENTRAL_VOWEL",
                    "a": "OPEN_FRONT_VOWEL",
                    "e": "OPEN_FRONT_VOWEL",
                    "E": "OPEN_FRONT_VOWEL",
                    "o": "OPEN_BACK_VOWEL",
                    "O": "OPEN_BACK_VOWEL",
                    "sil": "IDLE"}
        return

    def split_text(self, text):
        """Split too long sentence: handle by the Behavior Controller """
        if "." in text:
            text_array = text.split(".")
        else:
            text_array = []
            text_array.append(text)
        return text_array

    def split_behaviors(self, s):
        """Split the text from the behaviors """
        if len(s)>=2 and s[-1]=="*" and s[0]=="*":
            return [s]
        else:
            return re.split("\s+",s)

    def get_text_and_actions(self, sentence):
        """Get text and actions from the sentence """
        tokens = re.split("(\*[^\*\*]*\*)", sentence)
        phrase = ''.join(filter(lambda s: "*" not in s, tokens))
        rospy.loginfo("Processing the phrase: %s" %phrase)
        tokens = map(lambda s: self.split_behaviors(s), tokens)
        words = []
        for t in tokens:
            words += filter(lambda s: len(s) > 0, t)
        actions = []
        i = 0
        for w in words:
            if re.match("\*.*\*", w):
                args = w.strip("*").split()
                name = args.pop(0)
                actions.append([i,name,args])
            else:
                i += 1
        return (phrase, actions)

    def get_behaviors(self, response, actions):
        """Processing the response from AWS Polly and get the behaviors"""
        xSheet = []
        if "AudioStream" in response:
            with closing(response["AudioStream"]) as stream:
                    data = stream.read()
                    xSheet = data.split('\n')
                    xSheet = [json.loads(line) for line in xSheet if line != '']
        else:
            print("Could not stream audio")
        word_times = filter(lambda l: l["type"]=="word", xSheet)
        data=[]
        for w in word_times:
            data.append({"character":float(w["start"]) / 1000.,  # convert ms to seconds
                             "type":"word",
                             "start":float(w["time"]) / 1000.,
                             "value": str(w["value"])})
        for a in actions:
            if a[0] > len(word_times)-1:
                a[0] = xSheet[-1]["time"] / 1000.  # convert ms to seconds
            else:
                a[0] = (word_times[a[0]]["time"]) / 1000.  # convert ms to seconds
        for a in actions:
            args = a[2]
            data.append({"start":float(a[0])+.01, #prevent visemes and actions from being at exactly the same time
                         "type":"action",
                         "args":args,
                         "id": a[1]}) # End edits
        visemes = map(lambda l: [l["time"],self.vis_transl[l["value"]]], filter(lambda l: l["type"]=="viseme",xSheet))
        for v in visemes:
                data.append({"start":float(v[0]) / 1000.,  # convert ms to seconds
                             "type":"viseme",
                             "id": v[1]})
        return data

    def get_audio(self, response):
        """Get audio data from AWS Polly """
        outdir_path = self.outdir
        print(response)
        data={}
        data["file"] = outdir_path+"/tts.ogg"
        if "AudioStream" in response:
            with closing(response["AudioStream"]) as stream:
                output = data["file"]
            try:
                with open(output, "wb") as file:
                    file.write(stream.read())
            except IOError as error:
                    print(error)
        else:
            print("Could not stream audio")
        return data

    def get_response(self, behavior_data, audio_data):
        """ Get final response """
        behaviours = sorted(behavior_data, key = lambda i: i['start'])
        data, samplerate = sf.read(self.outdir + '/tts.ogg')
        sf.write(self.outdir + '/tts.wav', data, samplerate)
        file_handle = self.outdir + '/tts.wav'
        data = np.fromfile(file_handle, np.uint8)[self.wav_header_length:] #Loading wav file
        data = data.astype(np.uint8).tostring()
        data_array = data
        audio_frame = samplerate
        response = {
            "audio_frame" : audio_frame,
            "audio_data" : data_array,
            "behavior_data" : str(behaviours)
        }
        return str(response)

    def response_update(self, response_received, status, result_msg):
        super(AWSTtsService, self).update(response_received, status, result_msg)
        return

    def test(self):
        super(AWSTtsService, self).test()
        rospy.loginfo("Test the %s service" % self.name)
        success = True
        return success

    def request(self, input_text):
        rospy.loginfo("Start the %s request" % self.name)
        rate = "" #TODO: TBD
        super(AWSTtsService, self).request(rate)
        text = input_text
        [text, actions] = self.get_text_and_actions(text)
        try:
            text = '<speak><lang xml:lang="'+self.language+'">' + text + '</lang></speak>'
            json_response = self.tts.synthesize_speech(Text=text, TextType='ssml', OutputFormat="json", VoiceId=self.voice, SpeechMarkTypes =["viseme", "word"])
            behavior_data = self.get_behaviors(json_response, actions)
            ogg_response = self.tts.synthesize_speech(Text=text, TextType='ssml', OutputFormat="ogg_vorbis", VoiceId=self.voice)
            audio_data = self.get_audio(ogg_response)
            tts_response = self.get_response(behavior_data, audio_data)
            self.status = Status.RESPONSE_RECEIVED
            self.response_update(response_received=True, status=self.status, result_msg=tts_response["message"])
        except (BotoCoreError, ClientError) as error:
            rospy.logerr("The erros is " + str(error))
            self.start = Status.REQUEST_FAILED
            self.response_update(response_received=True, status=self.status, result_msg="")
        return


def main():
    try:
        service_name = "tts"
        rospy.init_node(service_name + "_node")
        last_event = ""  # TODO: How to get information about last_event from behavior controller?
        param = rospy.get_param("/"+service_name+"_param/")
        s = AWSTtsService(service_name, param)
        web_service_server = WebServiceServer(name=service_name, service_manager=s)
        web_service_server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

# Common Imports
import rospy

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace
from harmoni_tts.local_tts_client import TtsClient
from botocore.exceptions import BotoCoreError, ClientError
from contextlib import closing
import soundfile as sf
import numpy as np
import boto3
import re
import json
import sys


class LocalTtsService(HarmoniServiceManager):
    """
    Local TTS service using the TTS repository
    """

    def __init__(self, name, param):
        """Constructor method: Initialization of variables and model & config paths + setting up"""
        super().__init__(name)
        """Initialization of variables and TTS parameters"""
        self.tts_config = param["tts_config"]
        self.tts_model = param["tts_model"]
        self.vocoder_config = param["vocoder_config"]
        self.vocoder_model = param["vocoder_model"]
        self.use_cuda = param["use_cuda"]
        self.verbose = param["use_cuda"]
        self.speedup = param["use_cuda"]
        self.outdir = param["outdir"]

        """Initialize the local TTS client"""
        self.tts_client = TtsClient(
            self.tts_config,
            self.tts_model,
            self.vocoder_config,
            self.vocoder_model,
            self.use_cuda,
            self.verbose,
            self.speedup
        )

        """Setup the TTS service as server"""
        self.state = State.INIT
        return

    def request(self, input_text):
        """[summary]
        Args:
            input_text (str): Input string to synthesize
        Returns:
            object: It contains information about the response received (bool) and response message (str)
                response: bool
                message: str
        """
        rospy.loginfo("Start the %s request" % self.name)
        self.state = State.REQUEST

        try:
            alignment, mel_postnet_spec, stop_tokens, waveform = self.tts_client.get_audio(input_text)
            audio_data = self._get_audio(waveform)
        except Exception as e:
            rospy.logerr("The error is " + str(e))
            self.state = State.FAILED

        return {"response": self.state, "message": ""}

    def _get_audio(self, response):
        """[summary]
        This function writes the audio file getting data from TTS
        Args:
            response (obj): response from TTS for getting audio data

        Returns:
            data: audio data
        """
        data = {"file": self.outdir + "/tts.wav"}
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


def main():
    """[summary]
    Main function for starting local TTS service
    """
    service_name = ActuatorNameSpace.tts.name
    instance_id = rospy.get_param("instance_id")
    service_id = f"{service_name}_{instance_id}"
    try:
        rospy.init_node(service_name)

        param = rospy.get_param(service_name + "/" + instance_id + "_param/")

        s = LocalTtsService(service_id, param)

        service_server = HarmoniServiceServer(service_id, s)

        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

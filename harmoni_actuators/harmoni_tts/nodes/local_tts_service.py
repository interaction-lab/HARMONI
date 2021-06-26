#!/usr/bin/env python3

# Common Imports
import rospy

from harmoni_common_lib.constants import State, ActionType
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager

# Specific Imports
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import ActuatorNameSpace
from harmoni_tts.local_tts_client import TtsClient
from harmoni_common_msgs.msg import harmoniAction, harmoniGoal
import sounddevice as sd
import soundfile as sf


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
        self.scale_stats_path = param["scale_stats_path"]
        self.use_cuda = param["use_cuda"]
        self.verbose = param["use_cuda"]
        self.speedup = param["use_cuda"]
        self.outdir = param["outdir"]
        self.sample_rate = param["sample_rate"]

        """Initialize the local TTS client"""
        self.tts_client = TtsClient(
            self.tts_config,
            self.tts_model,
            self.vocoder_config,
            self.vocoder_model,
            self.scale_stats_path,
            self.use_cuda,
            self.verbose,
            self.speedup
        )
        instance_id = rospy.get_param("instance_id")
        name = ActuatorNameSpace.tts.name + "_" + instance_id
        self.speaker_action_client = HarmoniActionClient(name)

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
            file_path = self._save_audio_to_file(waveform)
            self.state = State.SUCCESS
        except Exception as e:
            rospy.logerr("The error is " + str(e))
            file_path = ""
            self.state = State.FAILED

        self.response_received = True

        return {"response": self.state, "message": file_path}

    def _save_audio_to_file(self, audio_data):
        """[summary]
        This function writes the audio data from TTS into a .wav file
        Args:
            audio_data (obj): response from TTS for getting audio data

        Returns:
            file_path: saved audio file
        """
        file_path = self.outdir + "/tts.wav"
        sf.write(
            file_path,
            audio_data,
            self.sample_rate
        )

        return file_path

    def publish_file_path(self, file_path):
        self.speaker_action_client.send_goal(
            action_goal=ActionType.DO.value,
            optional_data=file_path,
            wait=False
        )


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

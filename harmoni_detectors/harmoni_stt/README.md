# HARMONI STT

## Usage
Using the DeepSpeech service:
To set up the local STT service, first run `sh harmoni_detectors/harmoni_stt/get_deepspeech_models.sh` 
from the HARMONI directory in order to place the models in a parallel directory.

The local DeepSpeech speech-to-text service can be launched with `roslaunch harmoni_detectors stt_deepspeech_service.launch` or `roslaunch harmoni_detectors stt_service.launch service_to_launch:=deepspeech`.
Transcriptions are only published by the DeepSpeech service when the client determines the text as final based on the `t_wait` parameter (the default is 0.5s).

## Parameters
Parameters input for the local TTS service:
| Parameters           | Definition | Values |
|----------------------|------------|--------|
|model_file_path       |            |        |
|scorer_path           |            |        |
|lm_alpha              |            |        |
|lm_beta               |            |        |
|beam_width            |            |        |
|t_wait                |            |        |
|subscriber_id         |            |        |

## Testing
### Testing W2L 
Note: If running w2l, run get_w2l_models.sh before attempting to launch this service.

To test if the model has been built properly off of your microphone you can run the following in the terminal:

`ffmpeg -hide_banner -loglevel error -f alsa -i default -ar 16000 -ac 1 -ab 256k -f wav - | /root/wav2letter/build/inference/inference/examples/simple_streaming_asr_example --input_files_base_path=/root/model/w2l/`

`ffmpeg -hide_banner -loglevel error -f alsa -i hw:3 -ar 16000 -ac 1 -ab 256k -f wav - | /root/wav2letter/build/inference/inference/examples/simple_streaming_asr_example --input_files_base_path=/root/model/w2l/`

or

`roslaunch harmoni_detectors/harmoni_stt/launch/direct_stt_service.launch`

`ffmpeg -hide_banner -loglevel error -f alsa -i hw:3 -ar 16000 -ac 1 -ab 256k -f wav - | /root/wav2letter/build/inference/inference/examples/simple_streaming_asr_example --input_files_base_path=/root/model/w2l/`

`rostest harmoni_stt w2l.test --text`

To record your own test audio, use the following:

`arecord -r 16000 -d 6 -f S16_LE test_example.wav`

## References
[Documentation](https://harmoni.readthedocs.io/en/latest/packages/harmoni_stt.html)

https://trac.ffmpeg.org/wiki/Capture/ALSA

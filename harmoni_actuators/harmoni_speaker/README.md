# HARMONI Speaker

## Usage
The speaker device is configured in the dockerfiles/config/asoundrc file, whose contents are shown below:

```
pcm.!default {
  type plug
  slave {
    pcm "hw:0,0"
  }
}
ctl.!default {
  type hw
  card 1n
}
```

The speaker device is controlled by the hw values, currently listed as 0,0. To choose a speaker you may need to use the command ```aplay -L ``` to see what devices are available.
## Parameters
Parameters input for the speaker service: 

| Parameters           | Definition | Values |
|----------------------|------------|--------|
|chunk_size            |            |        |
|total_channels        |            |        |
|audio_rate            |            |        |

## Testing

To test that the speaker has been configured properly, use the command ```roslaunch harmoni_speaker speaker_service.launch``` which will play a short phrase through the configured device. You may need to experiement with different values to ensure the proper speaker is set up.

The speaker can also be tested by running `rostest harmoni_speaker speaker.test`. When being tested on a QT robot, this must be run on the QT's Raspberry Pi in order to hear the audio clip be played.
## References
[Documentation](https://harmoni.readthedocs.io/en/latest/packages/harmoni_speaker.html)

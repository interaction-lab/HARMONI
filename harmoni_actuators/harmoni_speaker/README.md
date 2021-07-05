# Speaker Service Parameters:
Parameters input for the speaker service: 

| Parameters           | Definition | Values |
|----------------------|------------|--------|
|chunk_size            |            |        |
|total_channels        |            |        |
|audio_rate            |            |        |

# Setting up your speaker
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

To test that the speaker has been configured properly, use the command ```roslaunch harmoni_speaker speaker_service_misty.launch test:=true``` which will play a short phrase through the configured device. You may need to experiement with different values to ensure the proper speaker is set up.
Be sure to add the correct robot_ip argument to the launch file if you want to test the case with no exception, or to add a mock one if you to test the exception.
 
# Speaker Service Parameters:
Parameters input for the speaker service: 

| Parameters           | Definition | Values |
|----------------------|------------|--------|
|chunk_size            |            |        |
|total_channels        |            |        |
|audio_rate            |            |        |

# Using Misty API
An argument robot_ip has been added to the launch file. In order to launch the service the Ip of the robot needs to be provided to the launch file as argument. In the speaker_service class, the ip_is saved.
In misty, the file must either be encoded in base64 or provided as file. 
To reproduce the file the flag ImmediatelyApply has been set to True in the API call.
The flag OverwriteExisting makes the new file replace the old, since the file is always called with the same name. 
Another possible way of doing this was to separate into two different API call, SaveAudio with ImmediatelyApply set to false and then PlayAudioClip.

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

To test that the speaker has been configured properly, use the command ```roslaunch harmoni_speaker speaker_service.launch test:=true``` which will play a short phrase through the configured device. You may need to experiement with different values to ensure the proper speaker is set up.

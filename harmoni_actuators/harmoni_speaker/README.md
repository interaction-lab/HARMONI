# Speaker Service Parameters:
Parameters input for the speaker service: 

| Parameters           | Definition | Values |
|----------------------|------------|--------|
|chunk_size            |            |        |
|total_channels        |            |        |
|audio_rate            |            |        |

# Using Misty API
An argument robot_ip is provided in the service_manager class. 
In misty, the file must either be encoded in base64 or provided as file. In the speaker_service_misty class the data are converted after being uploaded from a .wav file, either that they are provided to the class directly or via path. 

To reproduce the file the flag ImmediatelyApply has been set to True in the API call.
The flag OverwriteExisting makes the new file replace the old, since the file is always called with the same name. 
Another possible way of doing this was to separate into two different API call, SaveAudio with ImmediatelyApply set to false and then PlayAudioClip.

When replacing harmoni's speaker_service with speaker_service_misty, the type of action that the class performs has been changed from DO to REQUEST: this was done because conceptually we are requesting to an external service (the robot) to perform an action, and not performing the action directly inside HARMONI. This change requires to modify everything that interfaces with the speaker service 

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
# Gesture Service Parameters:
Parameters input for the gesture service: 

| Parameters           | Definition | Values |
|----------------------|------------|--------|
|chunk_size            |            |        |
|total_channels        |            |        |
|audio_rate            |            |        |

# Setting up your gesture
The gesture device is configured in the dockerfiles/config/asoundrc file, whose contents are shown below:

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

The gesture device is controlled by the hw values, currently listed as 0,0. To choose a gesture you may need to use the command ```aplay -L ``` to see what devices are available.

To test that the gesture has been configured properly, use the command ```roslaunch harmoni_gesture gesture_service.launch test:=true``` which will play a short phrase through the configured device. You may need to experiement with different values to ensure the proper gesture is set up.

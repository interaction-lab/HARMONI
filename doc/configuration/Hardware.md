# Hardware

*How to configure hardware and external endpoints that connect to HARMONI.*

## Audio

### Speaker

Check the `.asoundrc` file (located in `HARMONI/dockerfiles/config`) to ensure it refers to the appropriate sound device on your system. Determine which audio device you want to use by running `aplay -l` or `arecord -l` and update aforementioned `.asoundrc` file. *Note: make sure you can see hidden files*. If you are not using Docker, you will need to find or create the `.asoundrc` file in your home directory to configure sound in the same way.

```bash
cd dockerfiles/config
nano .asoundrc
# Update the hardware and soundcard id.
```
The contents are shown below:

```
pcm.!default {
  type plug
  slave {
    pcm "hw:0,0"
  }
}
ctl.!default {
  type hw
    card 0
}
```
  
The speaker device is controlled by the hw values, currently listed as 0,0 (the card `x` should be the same number of the first number in `hw:"x,y"`, and `y` refers to the device number).  
To choose a speaker you may need to use the command `aplay -l `  to see what devices are available.  
For example running `aplay -l`:
```
card 1: PCH [HDA Intel PCH], device 0: ALC283 Analog [ALC283 Analog]
```
In this case the `.asoundrc` file will be:
```
pcm.!default {
  type plug
  slave {
    pcm "hw:1,0"
  }
}
ctl.!default {
  type hw
    card 1
}
```

#### Testing Configuration

After updating your asoundrc and restarting docker test the audio with the following command:
```bash
rostest harmoni_speaker speaker.test
```
If it works, you're good! Otherwise, re-do your [audio config](#audio), and try the above command until the audio works.

### Microphone

Determine which capture audio device you want to use by running `arecord -l`.
To choose a microphone you may need to use the command `arecord -l `  to see what devices are available.  
For example running `arecord -l`:
```
card 1: PCH [HDA Intel PCH], device 0: ALC283 Analog [ALC283 Analog]
```
In this case, to make your microphone working, change the [configuration file](https://github.com/interaction-lab/HARMONI/blob/develop/harmoni_sensors/harmoni_microphone/config/configuration.yaml) of your `harmoni_microphone` package.
Generally, you need to modify only few parameters to get the microphone working. For example, check and substitute your _device_name_ and you microphone sample rate (_audio_rate_).

Look at the package README for further details!

#### Testing Configuration

After updating your configuration file and restarting docker test the audio with the following command:
```bash
rostest harmoni_microphone microphone.test
```
If it works, you're good! Otherwise, re-do your [audio config](#audio), and try the above command until the audio works.

## Video

Usually the defaults will work here. If you have trouble, verify which device you are pulling video from ([reference](https://askubuntu.com/a/848390)): 
```bash
sudo apt-get install v4l-utils
v4l2-ctl --list-devices
```
Then modify your docker-compose file and container (e.g. in `docker-compose.yml` under `harmoni_hardware:`) to select the desired video device. The pattern is `</local/devicepath>:<docker/devicepath>`. If you wanted to use `/dev/video1`, then you would replace the line under `devices:` which says `/dev/video0:/dev/video0` with `/dev/video1:/dev/video0` (if you would like to use multiple video devices, be sure to check the harmoni_camera config file).

### Camera

To work with your camera, again change the [configuration file](https://github.com/interaction-lab/HARMONI/blob/develop/harmoni_sensors/harmoni_camera/config/configuration.yaml) of your `harmoni_camera` package.You may need to change the _input_device_index_ (i.e., the index of your camera device plugged in), and the _video_format_ (i.e., how video format is encoded, for example "rgb8" or "bgr8").

Look at the package README for further details!

#### Testing Configuration

After updating your configuration file and restarting docker test the audio with the following command:
```bash
rostest harmoni_camera camera.test
```
If it works, you're good! Otherwise, re-do your [video config](#video), and try the above command until the audio works.


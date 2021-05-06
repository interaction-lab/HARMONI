# Configuration Setup

*How to configure hardware and external endpoints that connect to HARMONI.*

## Audio

Check the `.asoundrc` file (located in `HARMONI/dockerfiles/config`) to ensure it refers to the appropriate sound device on your system. Determine which audio device you want to use by running `aplay -l` or `arecord -l` and update aforementioned `.asoundrc` file. *Note: make sure you can see hidden files*.

## Video

Usually the defaults will work here. If you have trouble, verify which device you are pulling video from ([reference](https://askubuntu.com/a/848390)): 
```bash
sudo apt-get install v4l-utils
v4l2-ctl --list-devices
```
Then modify your docker-compose file (e.g. `docker-compose-full.yml`) to select the desired video device. If you wanted to use `/dev/video1`, then you would replace the line under `devices:` which says `/dev/video0:/dev/video0` with `/dev/video1:/dev/video`

## Cloud Services

This has mostly been covered in the Cloud-Services Section. In general, ensure you have stored your private keys/config/credentials in `~/.aws` or `~/.gcp`. The associated command line tools will usually do this for you.

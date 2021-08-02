# HARMONI Microphone

## Usage
## Parameters
Parameters input for the microphone service: 

| Parameters           | Definition | Values |
|----------------------|------------|--------|
|audio_format_width    |            |        |
|chunk_size            |            |        |
|total_channels        |            |        |
|audio_rate            |            |        |
|device_name           |            |        |


## Testing
The microphone can be tested by running `rostest harmoni_microphone microphone.test`. When being tested on a QT robot, this must be run on the QT's Raspberry Pi. This test checks that the microphone can record audio and will fail if the microphone has not been correctly set up or configured.
## References
[Documentation](https://harmoni.readthedocs.io/en/latest/packages/harmoni_microphone.html)
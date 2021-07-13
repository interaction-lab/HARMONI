# HARMONI TTS

The Text To Speech package takes text as action arguments and produces audio and vizemes for speech.

## Usage
## Parameters
Parameters input for the aws polly service: 

| Parameters           | Definition | Values |
|----------------------|------------|--------|
|region_name           |            |        |
|voice                 |            |        |
|language              |            |        |
|outdir                |            |        |
|wav_heade_length      |            |        |

## Testing
This package can be tested by running `rostest harmoni_tts polly.test`. Amazon Polly must be set up in order for this test to pass.
## References
[Documentation](https://harmoni.readthedocs.io/en/latest/packages/harmoni_tts.html)
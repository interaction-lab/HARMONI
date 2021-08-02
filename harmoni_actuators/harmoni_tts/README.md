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
This package can be tested by running `rostest harmoni_tts polly.test`. Amazon Polly must be set up in order for this test to pass. This test sends a request to connect to AWS services and will fail if the request does not succeed. It also sends text to the server to check that the text to speech services are working properly.
## References
[Documentation](https://harmoni.readthedocs.io/en/latest/packages/harmoni_tts.html)
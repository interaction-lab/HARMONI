# HARMONI TTS

The Text To Speech package takes text as action arguments and produces audio and vizemes for speech.

## Usage

When using the local tts run:

```sh setup_tts.sh```

Note: if using docker this will have already been run in building the container.

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

The local test will save a wav file of the speech specified in the tts.test test_tts_input parameter in the temp_data directory. 
## References
[Documentation](https://harmoni.readthedocs.io/en/latest/packages/harmoni_tts.html)

[Mozilla TTS](https://github.com/mozilla/TTS)
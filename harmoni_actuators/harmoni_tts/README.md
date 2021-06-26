# HARMONI TTS

The Text To Speech package takes text as action arguments and produces audio and vizemes for speech. 

## Usage
### Local TTS service
To set up the local TTS service, first run `sh setup_tts.sh` to install the local TTS models, configuration files, and dependencies.


## Parameters
Parameters input for the aws polly service: 

| Parameters           | Definition | Values |
|----------------------|------------|--------|
|region_name           |            |        |
|voice                 |            |        |
|language              |            |        |
|outdir                |            |        |
|wav_heade_length      |            |        |

Parameters input for the local TTS service:
| Parameters           | Definition | Values |
|----------------------|------------|--------|
|tts_config            |            |        |
|tts_model             |            |        |
|vocoder_config        |            |        |
|vocoder_model         |            |        |
|scale_stats_path      |            |        |
|use_cuda              |            |        |
|verbose               |            |        |
|speedup               |            |        |
|outdir                |            |        |
|sample_rate           |            |        |

## Testing
## References
[Documentation](https://harmoni.readthedocs.io/en/latest/packages/harmoni_tts.html)

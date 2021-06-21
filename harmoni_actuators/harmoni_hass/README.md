
## Home Assistant Service Parameters:
Parameters input for the home assistant service: 

| Parameters           | Definition | Values |
|----------------------|------------|--------|
|hass_uri              |   URL on which Home Assistant is available        |   e.g. "http://10.254.254.50:8123/"     |
|credential_path       |      path to the credentials file (see section below)     |  e.g. "$(env HOME)/.hass/token.json"  |
|simulation            |     pretend that a device has been on for too long (see "check_log" section)       |   True/False     |


INFO: We often refer to Home Assistant as "hass"

## Setting up your Home Assistant

Follow this guide to make your [Home Assistant setup](https://www.home-assistant.io/installation/)

Only Home Assistant Container has been tested (using Docker).

Make sure that Home Assistant is running before makeing requests to the harmoni_hass service.
 
### Set up credentials
The parameter *credential_path* refers to the file where the home assistant authentication token is stored.

The structure of this json file is: 
```
{
    "token" : "YOUR_HOME_ASSISTANT_TOKEN"
}
```

## REST API
[Home Assistant's REST API documentation](https://developers.home-assistant.io/docs/api/rest/)

IMPORTANT: entity_id is formed by the device domain and its name (e.g. entity_id for google home is media_player.google_home)

### Turn on/off a device
| Parameters           |Values |
|----------------------|--------|
|action              |     turn_on/turn_off       |
|entity_id       |    \<entity_id>        |
|answer (optional)            |     yes/no       |

Example: {"action":"turn_off", "entity_id":"switch.oven_power"}

Example with optional parameter: {"answer":"yes", "action":"turn_off", "entity_id":"oven_power", "type":"switch"}
If the answer parameter is set to "no", the action won't be executed. 

This command uses a POST API call to: /api/services/\<domain>/\turn_on or /api/services/\<domain>/\turn_off


### Play music from device
| Parameters           |Values |
|----------------------|--------|
|action              |     play_media       |
|entity_id       |    \<entity_id>        |
|media_content_id            |       \<audio_file>     |
|media_content_type            |     \<file_type>       |
|answer (optional)            |     yes/no       |

Example: {"action":"play_media", "entity_id": "media_player.googlehome8554", "media_content_id": "media-source://media_source/local/relax.wav", "media_content_type": "audio/wav", "answer":"yes"}

This command uses a POST API call to: /api/services/\<domain>/\play_media

To set the media source for Home Assistant see [here](https://www.home-assistant.io/integrations/media_source/) and [here](https://www.home-assistant.io/more-info/local-media/setup-media/).



### Check if an appliance is on for longer than X (default is 3 hours)
| Parameters           |Values |
|----------------------|--------|
|action              |     check_log       |
|entity       |    \<entity_id>        |
|answer (optional)            |     yes/no       |
|hours (optional)            |      \<integer>      |
|minutes (optional)            |     \<integer>        |
|seconds (optional)            |     \<integer>        |
|days (optional)            |     \<integer>        |

Example: "{ "action":"check_log", "entity":"switch.oven_power", "type":"switch", "hours" : "4", "answer":"yes"}"

This command uses a POST API call to: /api/logbook/\<timestamp>
timestamp is formatted like: 2021-05-24T10:00:00+00:00



## Info on how to create a HARMONI package
[How to create a HARMONI package](How_to_create_a_HARMONI_package.md)
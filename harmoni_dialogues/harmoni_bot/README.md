# HARMONI Bot

This package wraps different chatbot services that can be used with HARMONI. Currently we support AWS Lex, Google Dialogflow, and Rasa.

### Using the Rasa service
There are two default Rasa assistant available: `rasa_example` and `rasa_greeter`. The Rasa workspaces for these assistants are located in 
`harmoni_bot/src`, and can be set in `configuration.yaml` by changing the rasa_assistant parameter value to the name of the corresponding workspace:
"rasa_greeter" for `rasa_greeter`, "rasa_example" for `rasa_example`, etc. The start_rasa_server.sh script is called before
the service is launched and gets the correct path to train and run the Rasa server through rosparam.

### Adding Rasa assistants
Rasa workspaces must be placed in the `harmoni_bot` directory for the start_server script to find the workspace path. 

## Usage
## Parameters

Parameters input for the aws lex service: 

| Parameters           | Definition | Values |
|----------------------|------------|--------|
|user_id               |            |        |
|bot_name              |            |        |
|bot_alias             |            |        |
|region_name           |            |        |

Parameters input for the Rasa service: 

| Parameters           | Definition | Values |
|----------------------|------------|--------|
|rasa_assistant        |            |        |
|host                  |            |        |
|post                  |            |        |

## Testing
## References
[Documentation](https://harmoni.readthedocs.io/en/latest/packages/harmoni_bot.html)
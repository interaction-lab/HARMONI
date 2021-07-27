# HARMONI Bot

This package wraps different chatbot services that can be used with HARMONI. Currently we support AWS Lex, Google Dialogflow, and Rasa.

### Using the Rasa service
The default Rasa assistant is `rasa_example`. The Rasa workspace for this assistant is located in `harmoni_models/bot`, 
and the assistant name can be set in `configuration.yaml` by changing the rasa_assistant parameter value to the name of the corresponding workspace.

The start_rasa_server.sh script is called before the service is launched and gets the correct path to train and run the Rasa server through rosparam.
The model is only trained and run if it is a valid bot within `harmoni_models/bot`. 

### Adding Rasa assistants
Custom Rasa bots must also be placed in `harmoni_models/bot` for the start_server script to find the workspace path. 

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
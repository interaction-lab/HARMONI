# HARMONI Bot

This package wraps different chatbot services that can be used with HARMONI. Currently we support AWS Lex and Goodle Dialogflow. Rasa is a high priority on our roadmap for local chatbot functionality.

## Usage
## Parameters

Parameters input for the aws lex service: 

| Parameters           | Definition | Values |
|----------------------|------------|--------|
|user_id               |            |        |
|bot_name              |            |        |
|bot_alias             |            |        |
|region_name           |            |        |

## Testing
This package can be tested by running `rostest harmoni_bot lex.test`.

Amazon Lex should be set up in order for this test to pass.
## References
[Documentation](https://harmoni.readthedocs.io/en/latest/packages/harmoni_bot.html)
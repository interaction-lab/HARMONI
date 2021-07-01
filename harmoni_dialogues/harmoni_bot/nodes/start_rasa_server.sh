#!/bin/bash

RASA_ASSISTANT=$(rosparam get /rasa/default_param/rasa_assistant)
RASA_DIR=$(rosparam get /harmoni_bot_dir)

if [ "$RASA_ASSISTANT" == "rasa_example" ] || [ "$RASA_ASSISTANT" == "rasa_greeter" ]
  then
    cd "$RASA_DIR"/src/"$RASA_ASSISTANT"
    rasa train && rasa run
else
  echo "Not a valid Rasa bot"
fi
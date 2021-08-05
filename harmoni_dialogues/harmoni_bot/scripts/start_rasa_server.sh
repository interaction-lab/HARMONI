#!/bin/bash

shopt -s dotglob
shopt -s nullglob

RASA_ASSISTANT=$(rosparam get /rasa/default_param/rasa_assistant)
RASA_MODEL_DIR=$(rosparam get /bot_model_dir)
RASA_BOTS=("$RASA_MODEL_DIR/bot/"*)

found=0
for dir in "${RASA_BOTS[@]}" ; do
    dir=$(basename "$dir")
    [[ $dir = "$RASA_ASSISTANT" ]] && found=1
done

if [ $found == 1 ]; then
    cd "$RASA_MODEL_DIR"/bot/"$RASA_ASSISTANT" || exit
    rasa train && rasa run
else
    echo "Not a valid Rasa bot"
fi
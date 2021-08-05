#!/bin/bash

echo "This script should be run from the HARMONI directory in order to place the models in the harmoni_models package"

mkdir -p harmoni_models/stt && cd harmoni_models/stt
wget "https://github.com/mozilla/DeepSpeech/releases/download/v0.9.3/deepspeech-0.9.3-models.pbmm"
wget "https://github.com/mozilla/DeepSpeech/releases/download/v0.9.3/deepspeech-0.9.3-models.scorer"
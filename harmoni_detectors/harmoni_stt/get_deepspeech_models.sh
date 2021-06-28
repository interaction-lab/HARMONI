#!/bin/bash

echo "This script should be run from the HARMONI directory in order to place the models in a parallel directory"

cd ../..
mkdir -p model/deepspeech
cd model/deepspeech
wget -P models "https://github.com/mozilla/DeepSpeech/releases/download/v0.9.3/deepspeech-0.9.3-models.pbmm"
wget -P models "https://github.com/mozilla/DeepSpeech/releases/download/v0.9.3/deepspeech-0.9.3-models.scorer"
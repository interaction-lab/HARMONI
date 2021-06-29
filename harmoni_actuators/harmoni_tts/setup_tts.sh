#!/bin/bash

echo "This script should be run from the HARMONI directory in order to place the models in a parallel directory"

cd ../..
mkdir -p model/tts && cd model/tts \
  && gdown --id 1dntzjWFg7ufWaTaFy80nRz-Tu02xWZos -O tts_model.pth.tar \
  && gdown --id 18CQ6G6tBEOfvCHlPqP8EBI4xWbrr9dBc -O config.json \
  && gdown --id 1Ty5DZdOc0F7OTGj9oJThYbL5iVu_2G0K -O vocoder_model.pth.tar \
  && gdown --id 1Rd0R_nRCrbjEdpOwq6XwZAktvugiBvmu -O config_vocoder.json

cd ../../src/HARMONI/harmoni_actuators/harmoni_tts
gdown --id 11oY3Tv0kQtxK_JPgxrfesa99maVXHNxU -O scale_stats.npy
sudo apt-get install espeak
git clone https://github.com/coqui-ai/TTS
cd TTS
git checkout b1935c97
pip install -r requirements.txt
python setup.py install
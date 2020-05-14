#!/bin/bash

echo "This script should be run from the HARMONI directory in order to place the models in a parallel directory"

<<<<<<< HEAD:get_w2l_models.sh
cd /root/
=======
cd ../..
>>>>>>> 8497c8e... Misc. cleanup:harmoni_stt/get_w2l_models.sh
mkdir -p model/w2l
cd model/w2l
for f in acoustic_model.bin tds_streaming.arch decoder_options.json feature_extractor.bin language_model.bin lexicon.txt tokens.txt ; do wget http://dl.fbaipublicfiles.com/wav2letter/inference/examples/model/${f} ; done
ls
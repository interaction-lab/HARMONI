To pull audio for offline use with the current version of cordial_tts, which uses Ivona TTS, requires a couple extra steps, since Ivona dumps audio in the .ogg format rather than .wav, which is required by cordial_sound.

First, you'll need to install oggdec:

sudo apt-get install vorbis-tools

Then, to generate all the files in the proper formats, assuming you have a folder called "speech" in your package, in which you've put one or more script[1] files:

roscd <mypackage>/speech
mkdir data #skip this step if there's already a data directory in <mypackage>/speech

rosrun cordial_tts gen_phrases.py <Ivona voice name> ./data ./phrases.yaml ./<script filename 1> (./<script filename 2> ...)
oggdec ./data/*

You'll then need to provide the path to <mypackage>/speech/phrases.yaml as the "phrase_file" argument in your launch file, and set the argument "use_tts" to "False".


[1] A script file has one or more lines of the format (preserving both square and angle brackets):

[phrase_id] This is what <behavior_id>I want the robot to say.

where the phrase_id is a unique identifier for the phrase, and behavior_id is a behavior that your robot-specific implementation can handle (if using SPRITEbot, this is a behavior_id defined in the behaviors json file).  This implementation is fully compatible with defining dialogue trees that can be read by dialogue_manager.py in cordial_core.

 Child Voices: Ivy, Justin; Adult Voices: Salli, Joey, Kimberly, Kendra, Eric, Jennifer; Silly Voices: Chipmunk

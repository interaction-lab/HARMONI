# This is a script of example tests you can run in the command line

rostest harmoni_tts polly.test # tts

rostest harmoni_bot lex.test # bot

rostest harmoni_face face.test

rostest harmoni_web web.test

rostest harmoni_speaker speaker.test

rostest harmoni_stt w2l.test
rostest harmoni_stt deepspeech.test

rostest harmoni_face_detect face_detect.test

rostest harmoni_camera camera.test

rostest harmoni_microphone microphone.test

# harmoni_decision launcher.launch service:='harmoni,hardware'

roslaunch harmoni_tts tts_service.launch test:=true

roslaunch harmoni_bot bot_service.launch test:=true

roslaunch harmoni_face face_service.launch test:=true

roslaunch harmoni_web web_service.launch test:=true

roslaunch harmoni_speaker speaker_service.launch test:=true

roslaunch harmoni_stt stt_service.launch test:=true

roslaunch harmoni_face_detect face_detect_service.launch test:=true

roslaunch harmoni_camera camera_service.launch test:=true

roslaunch harmoni_microphone microphone_service.launch test:=true

harmoni_decision launcher.launch service:='harmoni,hardware'
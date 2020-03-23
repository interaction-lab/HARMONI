#!/usr/bin/env python

#------------------------------------------------------------------------------
# CoRDial Player, a ROS node for playing time-synchronized speech and behavior
# Copyright (C) 2017 Elaine Schaertl Short
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#------------------------------------------------------------------------------

import roslib; roslib.load_manifest('cordial_core')
import rospy
import actionlib
from cordial_core.msg import *
from cordial_sound.msg import SoundRequest
from cordial_tts import CoRDialTTS
from cordial_sound.libsoundplay import SoundClient
from cordial_face.msg import FaceRequest

import yaml
import sys
import wave
import contextlib
import argparse
from threading import Timer

class PlayerServer():
    _feedback = PlayerFeedback()
    _result = PlayerResult()

    def __init__(self, phone_face, delay=0.0,phrase_file=None, use_tts=False, voice=None):
        self._use_tts = use_tts in ['true', 'True', 'TRUE', '1']

        self.phrase_file = phrase_file

        if phrase_file and not len(phrase_file)>0 and not use_tts:
            rospy.logerr("CoRDial Player Error: Must specify phrase file or allow tts! Continuing without sound...")
            self._sound = False
        elif not phrase_file and not use_tts:
            rospy.logerr("CoRDial Player Error: Must specify phrase file or allow tts! Continuing without sound...")
            self._sound = False
        # elif (not phrase_file or len(phrase_file)>0) and use_tts and (not ivona_secret_key or not ivona_access_key):
        #     rospy.logerr("CoRDial Player Error: Must specify ivona keys to allow tts! Continuing without sound...")
        #     self._sound = False
        #     self._use_tts=False
        else:
            self._sound = True



        self._speech_delay_time = delay
        self._phone_face = phone_face

        self._phrases=None
        if phrase_file and len(phrase_file)>0:
            self._sound_client = SoundClient()
            rospy.sleep(0.5)
            self._sound_client.stopAll()

            rospy.loginfo("CoRDial Player reading Phrase File... this could take a while")
            with open(phrase_file, 'r') as f:
                s = f.read()
                self._phrases = yaml.load(s)
            rospy.loginfo("Phrase file read.")

        if self._use_tts:
            self._tts = CoRDialTTS(voice)

        base_topic = ""

        self._behavior_client = actionlib.SimpleActionClient(base_topic+'Behavior',BehaviorAction)
        if self._phone_face:
            self._face_pub = rospy.Publisher(base_topic+'face', FaceRequest, queue_size=10)
            rospy.sleep(0.5)
        rospy.loginfo("CoRdial Player waiting for behavior server..")
        self._behavior_client.wait_for_server()
        rospy.loginfo("CoRDial Player connected to behavior server")

        self._feedback.behavior = "none"

        rospy.loginfo("Starting CoRDial Player server...")
        self._server = actionlib.SimpleActionServer(base_topic+'Player', PlayerAction, execute_cb=self.execute_cb,auto_start=False)

        info = ""
        if self._phone_face:
            info += ", using CoRDial face"
        else:
            info += ", NOT using CoRDial face"
        if self._use_tts:
            info += " and using TTS with voice " + voice
        elif self._phrases:
            info += " and NOT using TTS"
        if self._phrases:
            info += "; phrase file is " + phrase_file
        if not self._use_tts and not self._phrases:
            info+= " and with NO sound"


        info += ". Delay time is " + str(self._speech_delay_time) + "s."

        self._server.start()
        rospy.loginfo("CoRDial Player server started"+ info)


    def execute_cb(self, goal):
        rospy.loginfo("Phrase playing: " + str(goal.phrase))
        preempted = False

        if goal.interrupt == True:
            rospy.loginfo("Stopping audio")
            if self._phrases:
                self._sound_client.stopAll()
            if self._use_tts:
                self._tts.shutup()
            #TODO: more intelligent goal cancelling?
            rospy.loginfo("Stopping behaviors")
            self._behavior_client.cancel_all_goals()


        phrase_found = False
        if self._phrases:
            try:
                #added for a second
                if self._use_tts:
                    with open(self.phrase_file, 'r') as f:
                        s = f.read()
                        self._phrases = yaml.load(s)


                behaviors = self._phrases[goal.phrase]["behaviors"]
                phrase_found = True
            except KeyError as k:
                if not self._use_tts:
                    rospy.logerr("Phrase id %s not recognized"%goal.phrase)
                    rospy.logerr(str(type(k)))
                    rospy.logerr(str(k.args))
                    self._result.result = "FAILED"
                    self._server.set_aborted(self._result)
                    return
                else:
                    phrase_found = False
                    rospy.logwarn("Phrase id %s not recognized; using TTS"%goal.phrase)
        if not phrase_found:
            time1 = rospy.Time.now()
            phrase,behaviors=self._tts.extract_behaviors(goal.phrase)

        if self._phone_face:
            # visemes = ['AO_AW', 'CH_SH_ZH', 'R_ER', 'L', 'IDLE', 'AA_AH', 'EY', 'M_B_P', 'N_NG_D_Z', 'EH_AE_AY', 'OO', 'F_V']
            visemes = ["BILABIAL","LABIODENTAL","INTERDENTAL","DENTAL_ALVEOLAR","POSTALVEOLAR","VELAR_GLOTTAL","CLOSE_FRONT_VOWEL","OPEN_FRONT_VOWEL","MID_CENTRAL_VOWEL","OPEN_BACK_VOWEL","CLOSE_BACK_VOWEL", 'IDLE']
            viseme_behaviors = filter(lambda b: b["id"] in visemes, behaviors)
            behaviors = filter(lambda b: b["id"] not in visemes, behaviors)


            min_duration=0.05
            for i in range(0,len(viseme_behaviors)-1):
                viseme_behaviors[i]["duration"]=viseme_behaviors[i+1]["start"]-viseme_behaviors[i]["start"]
            viseme_behaviors[-1]["duration"]=min_duration
            viseme_behaviors=filter(lambda b: b["duration"]>= min_duration, viseme_behaviors)

            ordered_visemes = sorted(viseme_behaviors, key=lambda b: b["start"])
            viseme_ids = map(lambda b: b["id"], ordered_visemes)
            viseme_times = map(lambda b: b["start"], ordered_visemes)
            viseme_speed = 10
            # viseme_speed = map(lambda b: b["duration"], ordered_visemes)

            viseme_req = FaceRequest(visemes=viseme_ids, viseme_ms=viseme_speed, times=viseme_times)

        ordered_behaviors = sorted(behaviors,
                                 key=lambda behavior: behavior["start"])


        speech_delay_time = self._speech_delay_time

        #for behaviors
        if speech_delay_time < 0:
            timing_adjust = rospy.Duration.from_sec(0-speech_delay_time)
            speech_delay_time=0
        else:
            timing_adjust = rospy.Duration.from_sec(0)


        self.speech_duration = 0.0
        self.speech_start_time=rospy.Time.now()

        if phrase_found:
            wave_file = self._phrases[goal.phrase]["file"]
            with contextlib.closing(wave.open(wave_file,'r')) as f:
                frames=f.getnframes()
                rate=f.getframerate()
                self.speech_duration=frames/float(rate)
                self.speech_start_time=rospy.Time.now()
            def speak():
                self._sound_client.playWave(wave_file)
                rospy.loginfo("Speech: playing wave file -- duration: " + str(self.speech_duration))
            t = Timer(speech_delay_time, speak)
            t.start()

        else:
            def speak():
                ogg_file = self._tts.say(phrase)
                self.speech_duration = 3.0
                self.speech_start_time=rospy.Time.now()
                rospy.loginfo("Speech: playing tts speech -- duration: " + str(self.speech_duration))
                self._sound_client.playWave(ogg_file)
            t = Timer(speech_delay_time, speak)
            t.start()

        if self._phone_face:
            viseme_delay_time = timing_adjust.to_sec()

            def send_visemes():
                self._face_pub.publish(viseme_req)
            t = Timer(viseme_delay_time, send_visemes)
            t.start()

	start_time = rospy.Time.now()
        for a in ordered_behaviors:
            rospy.loginfo("Playing behavior: " + str(a))
            while rospy.Time.now()-start_time < rospy.Duration.from_sec(a["start"])+timing_adjust and not self._server.is_preempt_requested():
                pass
            if self._server.is_preempt_requested():
                preempted = True
                break


            if_conflict = BehaviorGoal.DROP
            if "interrupt" in a.keys():
                if_conflict = BehaviorGoal.OVERRIDE

            if "args" in a.keys():
                args = a["args"]
            else:
                args = []

            if "hold" in a.keys():
                hold = bool(a["hold"])
            else:
                hold = True # default is yes, hold pose until told otherwise

            if "move_time" in a.keys():
                end_move = rospy.Time.now()+rospy.Duration.from_sec(a["move_time"])
            else:
                end_move = rospy.Time()

            if "end" in a.keys():
                end_hold = rospy.Duration.from_sec(a["end"])+start_time
            else:
                end_hold = rospy.Time()

            goal = BehaviorGoal(behavior=a["id"], end_hold=end_hold, end_move=end_move, return_to_prior=not hold, if_conflict = if_conflict, args = args, wait_and_block = False) #old version

            self._behavior_client.send_goal(goal) #old version
            self._feedback.behavior = a["id"]
            self._server.publish_feedback(self._feedback)

        if preempted:
            rospy.loginfo("Preempted")
            self._behavior_client.cancel_all_goals()

            if phrase_found:
                self._sound_client.stopAll()
            else:
                self._tts.shutup()
            self._server.set_preempted()
            return
        else:
            rospy.loginfo("Waiting for end")
            while (rospy.Time.now()-self.speech_start_time) < rospy.Duration.from_sec(self.speech_duration)+rospy.Duration.from_sec(speech_delay_time):
                pass
            rospy.loginfo("At end -- Success")
            self._result.result = PlayerResult.DONE
            self._server.set_succeeded(self._result)
            return
        rospy.logwarn("Speech: the code should probably never reach this point. Setting succeeded anyway.")
        self._result.result = PlayerResult.DONE
        self._server.set_succeeded(self._result)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Core audio + behavior playback of CoRDial')
    parser.add_argument('-f', '--use-face', help='Collect visemes and send to cordial_face', action='store_true')
    parser.add_argument('-v', '--voice', help="Which voice to use with TTS. Child Voices: Ivy, Justin; Adult Voices: Salli, Joey, Kimberly, Kendra, Eric, Jennifer; Silly Voices: Chipmunk", default="Ivy")
    parser.add_argument('-k1', '--ivona-access-key', help="Ivona access key", nargs='?', default=None)
    parser.add_argument('-k2', '--ivona-secret-key', help="Ivona secret key", nargs='?', default=None)
    parser.add_argument('-d', '--delay', help="How much should the speech be delayed by, in s", default=0.0, type=float)
    parser.add_argument('-p', '--phrase-file', help="Phrase file for pre-loaded speech", nargs="?", default=None)
    parser.add_argument('-t', '--use-tts', help="Enable text-to-speech (online)", default=False)
    args = parser.parse_known_args()[0]

    rospy.init_node('cordial_player')

    PlayerServer(args.use_face, args.delay, args.phrase_file, args.use_tts, args.voice)

    while not rospy.is_shutdown():
        rospy.spin()

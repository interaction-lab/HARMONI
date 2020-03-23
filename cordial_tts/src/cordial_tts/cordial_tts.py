#!/usr/bin/env python

#------------------------------------------------------------------------------
# Interface between CoRDial and Amazon Polly
# Copyright (C) 2018 Elaine Schaertl Short and Nathaniel Steele Dennler
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



import roslib; roslib.load_manifest('cordial_tts')
import sys
import re
import os
import json
from boto3 import client
from botocore.exceptions import BotoCoreError, ClientError
from contextlib import closing

class CoRDialTTS():
    def __init__(self, voice):
        self.voice = voice;
        self.tts = client("polly", region_name='us-west-1')# you can change the region in this line

    #function used to generate viseme and expression time data.
    #input - line of text to be processed with polly
    def extract_behaviors(self,line):
        vis_transl = {"p": "BILABIAL",
                    "f": "LABIODENTAL",
                    "T": "INTERDENTAL",
                    "s": "DENTAL_ALVEOLAR",
                    "t": "DENTAL_ALVEOLAR",
                    "S": "POSTALVEOLAR",
                    "r": "POSTALVEOLAR",
                    "k": "VELAR_GLOTTAL",
                    "i": "CLOSE_FRONT_VOWEL",
                    "u": "CLOSE_BACK_VOWEL",
                    "@": "MID_CENTRAL_VOWEL",
                    "a": "OPEN_FRONT_VOWEL",
                    "e": "OPEN_FRONT_VOWEL",
                    "E": "OPEN_FRONT_VOWEL",
                    "o": "OPEN_BACK_VOWEL",
                    "O": "OPEN_BACK_VOWEL",
                    "sil": "IDLE"}

        #filter out the emotes surrounded by asterisks
        tokens = re.split("(\*[^\*\*]*\*)", line)
        phrase = ''.join(filter(lambda s: "*" not in s, tokens))


        def cond_split(s):
            if len(s)>=2 and s[-1]=="*" and s[0]=="*":
                return [s]
            else:
                return re.split("\s+",s)

        tokens = map(lambda s: cond_split(s), tokens)
        words = []

        for t in tokens:
            words += filter(lambda s: len(s) > 0, t)

        actions = []
        i = 0
        for w in words:
            if re.match("\*.*\*", w):
                args = w.strip("*").split()
                name = args.pop(0)
                actions.append([i,name,args])
            else:
                i += 1
        print("PROCESSING: '{}'".format(phrase)) #print out the phase to be converted to speech

        #example server code for reference: https://docs.aws.amazon.com/polly/latest/dg/example-Python-server-code.html
        #use the interface to communicate with the Amazon Polly Client
        try:
            #enclose phrase in speak tags to use Polly's ssml, and let Polly know the language
            phrase = '<speak><lang xml:lang="en-US">' + phrase + '</lang></speak>'
    	    response = self.tts.synthesize_speech(Text=phrase, TextType='ssml', OutputFormat="json", VoiceId=self.voice, SpeechMarkTypes =["viseme", "word"])
        except (BotoCoreError, ClientError) as error:
    	    print(error)
    	    sys.exit(-1)

        #get json of times to play visemes and words (called an exposure sheet or x-sheet in animation)
    	xSheet = []
    	if "AudioStream" in response:
    	    with closing(response["AudioStream"]) as stream:
                    data = stream.read()
                    xSheet = data.split('\n')
                    xSheet = [json.loads(line) for line in xSheet if line != '']
    	else:
    	    print("Could not stream audio")
    	    sys.exit(-1)

        #find the times to play beginnings of words, so the actions can be spliced in.
    	word_times = filter(lambda l: l["type"]=="word", xSheet)
        #data will be the list of visemes and actions in order by time
        #data will collect also information about word timing
        data=[]
        for w in word_times:
            data.append({"character":float(w["start"]) / 1000.,  # convert ms to seconds
                             "type":"word",
                             "start":float(w["time"]) / 1000.,
                             "value": str(w["value"])})
        
        #assign the actions the correct time based on when they appear in the script
        for a in actions:
            if a[0] > len(word_times)-1:
                a[0] = xSheet[-1]["time"] / 1000.  # convert ms to seconds
            else:
	        a[0] = (word_times[a[0]]["time"]) / 1000.  # convert ms to seconds
        #data will contain also the information about the timing of the words

        
    	
        for a in actions:
            args = a[2]
            data.append({"start":float(a[0])+.01, #prevent visemes and actions from being at exactly the same time
                         "type":"action",
                         "args":args,
			             "id": a[1]}) # End edits

        #get the times of just the visemes, and convert the viseme keys to the set used by CoRDial.
    	visemes = map(lambda l: [l["time"],vis_transl[l["value"]]], filter(lambda l: l["type"]=="viseme",xSheet))
    	for v in visemes:
                data.append({"start":float(v[0]) / 1000.,  # convert ms to seconds
                             "type":"viseme",
                             "id": v[1]})

        return phrase, data

    def phrase_to_file(self,name, line, outdir):
        outdir_path = os.path.abspath(os.path.expanduser(outdir))

        data={}
        phrase, data["behaviors"]=self.extract_behaviors(line)

        data["file"] = outdir_path+"/"+name+".ogg"
        data["text"] = '"'+phrase+'"'

        try:
    	    response = self.tts.synthesize_speech(Text=phrase, TextType='ssml', OutputFormat="ogg_vorbis", VoiceId=self.voice)
        except (BotoCoreError, ClientError) as error:
    	    print(error)
    	    sys.exit(-1)

    	if "AudioStream" in response:
    	    with closing(response["AudioStream"]) as stream:
    		output = data["file"]
    		try:
    		    with open(output, "wb") as file:
    			file.write(stream.read())
    		except IOError as error:
    		    	print(error)
    			sys.exit(-1)

    	else:
    	    print("Could not stream audio")
    	    sys.exit(-1)

        return data

    def say(self,phrase, wait=False, interrupt=False):
        return self.tts.synthesize_speech(Text=phrase, TextType='ssml', OutputFormat="ogg_vorbis", VoiceId=self.voice)

    def is_speaking(self):
        pass
        # self.tts.is_busy()

    def shutup(self):
        pass
        # self.tts.shutup()

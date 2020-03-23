#!/usr/bin/env python

#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  Modifications (c) 2017, Elaine Schaertl Short
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

# Author: Blaise Gassend

import roslib; roslib.load_manifest('cordial_sound')

import rospy
import threading
from cordial_sound.msg import SoundRequest
import os
import logging
import sys
import traceback
import tempfile
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue, DiagnosticArray

try:
    # import pygst
    # pygst.require('0.10')
    # import gst
    # import gobject
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import GObject, Gst

except:
    str="""
**************************************************************
Error opening pygst. Is gstreamer installed? (sudo apt-get install python-gst0.10
**************************************************************
"""
    rospy.logfatal(str)
    print str
    exit(1)

def sleep(t):
    try:
        rospy.sleep(t)
    except:
        pass

'''
Class soundtype responsible for calling Gst to play sounds
    fields: lock - locking mechanism
            state - keeps track of what the player is currently doing
            sound - the GST player object
            uri - path to the sound file to play
            volume - number tracking the current volume the file is being played at
            staleness - how old the player is
            file - the wav / ogg file being played

    methods:command() - general purpose method to do something with a file (play, stop, loop, etc.)
            setVolume() - changes the volume of a file
            loop() - plays a file repeatedly
            stop() - stops playing a file
            single() - plays a file through once
            getStaleness() - queries duration and position, and determines
                             how long a file has been finished playing for
'''
class soundtype:
    STOPPED = 0
    LOOPING = 1
    COUNTING = 2

    def __init__(self, file, volume = 1.0):
        self.lock = threading.RLock()
        self.state = self.STOPPED

        GObject.threads_init()
        Gst.init(None)
        self.sound = Gst.ElementFactory.make("playbin", "player")
        if not self.sound:
            sys.stderr.write("'playbin' gstreamer plugin missing\n")
            sys.exit(1)
        # self.sound = gst.element_factory_make("playbin","player")
        if (":" in file):
            uri = file
        elif os.path.isfile(file):
            uri = "file://" + os.path.abspath(file)
        else:
          rospy.logerr('Error: URI is invalid: %s'%file)

        self.uri = uri
        self.volume = volume
        self.sound.set_property('uri', uri)
        self.sound.set_property("volume",volume)
        self.staleness = 1
        self.file = file

    def __del__(self):
        # stop our GST object so that it gets garbage-collected
        self.stop()

    def command(self, cmd, arg = 1.0):
         if cmd == SoundRequest.PLAY_STOP:
             self.stop()
         elif cmd == SoundRequest.PLAY_ONCE:
             self.single()
         elif cmd == SoundRequest.PLAY_START:
             self.loop()
         elif cmd == SoundRequest.CHANGE_VOL:
             self.set_volume(arg)

    def set_volume(self, vol):
        if vol > 1.0:
            rospy.logwarn('Volume value too large, changing to max value (1.0)')
            vol = 1.0
        if vol < 0:
            rospy.logwarn('Volume value too small, changing to min value (0.0)')
            vol = 0
        self.lock.acquire()
        try:
            self.volume = vol
            self.sound.set_property("volume", vol)
        finally:
            self.lock.release()

    def loop(self):
        self.lock.acquire()
        try:
            self.staleness = 0
            if self.state == self.COUNTING:
                self.stop()

            if self.state == self.STOPPED:
              self.sound.seek_simple(Gst.Format.TIME, Gst.SeekFlags.FLUSH, 0)
              # self.sound.set_state(gst.STATE_PLAYING)
              self.sound.set_state(Gst.State.PLAYING)

            self.state = self.LOOPING
        finally:
            self.lock.release()

    def stop(self):
        if self.state != self.STOPPED:
            self.lock.acquire()
            try:
                self.sound.set_state(Gst.State.NULL)
                self.state = self.STOPPED
            finally:
                self.lock.release()

    def single(self):
        self.lock.acquire()
        try:
            rospy.logdebug("Playing %s"%self.uri)
            self.staleness = 0
            if self.state == self.LOOPING:
                self.stop()

            self.sound.seek_simple(Gst.Format.TIME, Gst.SeekFlags.FLUSH, 0)
            self.sound.set_state(Gst.State.PLAYING)

            self.state = self.COUNTING
        finally:
            self.lock.release()

    def get_staleness(self):
        self.lock.acquire()
        position = 0
        duration = 0
        try:
            position = self.sound.query_position(Gst.Format.TIME)[0]
            duration = self.sound.query_duration(Gst.Format.TIME)[0]
        except Exception, e:
            position = 0
            duration = 0
        finally:
            self.lock.release()

        if position != duration:
            self.staleness = 0
        else:
            self.staleness = self.staleness + 1
        return self.staleness

'''
class soundplay responsible for determining where the soundfile will be coming from
    files can come from three main sources: local, generated on-the-fly, or built-in.
        local files - use the SoundRequest type PLAY_FILE
        generated files use the SoundRequest type SAY
        built-in files are the fallback case

'''
class soundplay:
    def stopdict(self,dict):
        for sound in dict.values():
            sound.stop()

    def stopall(self):
        self.stopdict(self.builtinsounds)
        self.stopdict(self.filesounds)
        self.stopdict(self.voicesounds)

    def callback(self,data):
        if not self.initialized:
            return
        self.mutex.acquire()

        # Force only one sound at a time
        # self.stopall()
        try:
            #immediately stop sound if that is the command being issued
            if data.sound == SoundRequest.ALL and data.command == SoundRequest.PLAY_STOP:
                self.stopall()
            else:
                #determine the the source of the file
                if data.sound == SoundRequest.PLAY_FILE:
                    #cache the file or read from the cache
                    if not data.arg in self.filesounds.keys():
                        rospy.logdebug('command for uncached wave: "%s"'%data.arg)
                        try:
                            self.filesounds[data.arg] = soundtype(data.arg)
                        except:
                            rospy.logerr( "Exception:")
                            print(sys.exc_info()[0])
                            rospy.logerr('Error setting up to play "%s". Does this file exist on the machine on which cordial_sound is running?'%data.arg)
                            return
                    else:
                        print "cached"
                        rospy.logdebug('command for cached wave: "%s"'%data.arg)
                    sound = self.filesounds[data.arg]

                # if sound has not been recorded, get a wav from the text given
                elif data.sound == SoundRequest.SAY:
                    if not data.arg in self.voicesounds.keys():
                        rospy.logdebug('command for uncached text: "%s"' % data.arg)
                        txtfile = tempfile.NamedTemporaryFile(prefix='cordial_sound', suffix='.txt')
                        (wavfile,wavfilename) = tempfile.mkstemp(prefix='cordial_sound', suffix='.wav')
                        txtfilename=txtfile.name
                        os.close(wavfile)
                        voice = data.arg2
                        try:
                            txtfile.write(data.arg)
                            txtfile.flush()
                            os.system("text2wave -eval '("+voice+")' "+txtfilename+" -o "+wavfilename)
                            try:
                                if os.stat(wavfilename).st_size == 0:
                                    raise OSError # So we hit the same catch block
                            except OSError:
                                rospy.logerr('Sound synthesis failed. Is festival installed? Is a festival voice installed? Try running "rosdep satisfy cordial_sound|sh". Refer to http://pr.willowgarage.com/wiki/cordial_sound/Troubleshooting')
                                return
                            self.voicesounds[data.arg] = soundtype(wavfilename)
                        finally:
                            txtfile.close()
                    else:
                        rospy.logdebug('command for cached text: "%s"'%data.arg)
                    sound = self.voicesounds[data.arg]

                #Otherwise, just play one of the built in sounds
                else:
                    rospy.logdebug('command for builtin wave: %i'%data.sound)
                    if not data.sound in self.builtinsounds:
                        params = self.builtinsoundparams[data.sound]
                        self.builtinsounds[data.sound] = soundtype(params[0], params[1])
                    sound = self.builtinsounds[data.sound]


                if sound.staleness != 0 and data.command != SoundRequest.PLAY_STOP:
                    # This sound isn't counted in active_sounds
                    rospy.logdebug("activating %i %s"%(data.sound,data.arg))
                    self.active_sounds = self.active_sounds + 1
                    sound.staleness = 0
#                    if self.active_sounds > self.num_channels:
#                        mixer.set_num_channels(self.active_sounds)
#                        self.num_channels = self.active_sounds

                if(data.command == SoundRequest.CHANGE_VOL):
                    try:
                        volume = float(data.arg2)
                    except:
                        rospy.logerr("Error getting volume from message")
                        return
                    sound.command(data.command, volume)
                else:
                    sound.command(data.command)
        except Exception, e:
            rospy.logerr('Exception in callback: %s'%str(e))
            rospy.loginfo(traceback.format_exc())
        finally:
            self.mutex.release()
            rospy.logdebug("done callback")

    # Purge sounds that haven't been played in a while.
    def cleanupdict(self, dict):
        purgelist = []
        for (key,sound) in dict.iteritems():
            try:
                staleness = sound.get_staleness()
            except Exception, e:
                rospy.logerr('Exception in cleanupdict for sound (%s): %s'%(str(key),str(e)))
                staleness = 100 # Something is wrong. Let's purge and try again.
            #print "%s %i"%(key, staleness)
            if staleness >= 10:
                purgelist.append(key)
            if staleness == 0: # Sound is playing
                self.active_sounds = self.active_sounds + 1
        for key in purgelist:
           rospy.logdebug('Purging %s from cache'%key)
           del dict[key]

    def cleanup(self):
        self.mutex.acquire()
        try:
            self.active_sounds = 0
            self.cleanupdict(self.filesounds)
            self.cleanupdict(self.voicesounds)
            self.cleanupdict(self.builtinsounds)
        except:
            rospy.loginfo('Exception in cleanup: %s'%sys.exc_info()[0])
        finally:
            self.mutex.release()

    def diagnostics(self, state):
        try:
            da = DiagnosticArray()
            ds = DiagnosticStatus()
            ds.name = rospy.get_caller_id().lstrip('/') + ": Node State"
            if state == 0:
                ds.level = DiagnosticStatus.OK
                ds.message = "%i sounds playing"%self.active_sounds
                ds.values.append(KeyValue("Active sounds", str(self.active_sounds)))
                ds.values.append(KeyValue("Allocated sound channels", str(self.num_channels)))
                ds.values.append(KeyValue("Buffered builtin sounds", str(len(self.builtinsounds))))
                ds.values.append(KeyValue("Buffered wave sounds", str(len(self.filesounds))))
                ds.values.append(KeyValue("Buffered voice sounds", str(len(self.voicesounds))))
            elif state == 1:
                ds.level = DiagnosticStatus.WARN
                ds.message = "Sound device not open yet."
            else:
                ds.level = DiagnosticStatus.ERROR
                ds.message = "Can't open sound device. See http://pr.willowgarage.com/wiki/cordial_sound/Troubleshooting"
            da.status.append(ds)
            da.header.stamp = rospy.get_rostime()
            self.diagnostic_pub.publish(da)
        except Exception, e:
            rospy.loginfo('Exception in diagnostics: %s'%str(e))

    def __init__(self):
        rospy.init_node('cordial_sound')
        self.diagnostic_pub = rospy.Publisher("/CoRDial/sound/diagnostics", DiagnosticArray, queue_size=10)

        rootdir = os.path.join(os.path.dirname(__file__),'..','sounds')

        self.builtinsoundparams = {
                SoundRequest.BACKINGUP              : (os.path.join(rootdir, 'BACKINGUP.ogg'), 0.1),
                SoundRequest.NEEDS_UNPLUGGING       : (os.path.join(rootdir, 'NEEDS_UNPLUGGING.ogg'), 1),
                SoundRequest.NEEDS_PLUGGING         : (os.path.join(rootdir, 'NEEDS_PLUGGING.ogg'), 1),
                SoundRequest.NEEDS_UNPLUGGING_BADLY : (os.path.join(rootdir, 'NEEDS_UNPLUGGING_BADLY.ogg'), 1),
                SoundRequest.NEEDS_PLUGGING_BADLY   : (os.path.join(rootdir, 'NEEDS_PLUGGING_BADLY.ogg'), 1),
                }

        self.mutex = threading.Lock()
        sub = rospy.Subscriber("/CoRDial/robotsound", SoundRequest, self.callback)
        self.mutex.acquire()
        self.no_error = True
        self.initialized = False
        self.active_sounds = 0
        self.sleep(0.5) # For ros startup race condition
        self.diagnostics(1)

        while not rospy.is_shutdown():
            while not rospy.is_shutdown():
                self.init_vars()
                self.no_error = True
                self.initialized = True
                self.mutex.release()
                try:
                    self.idle_loop()
                    # Returns after inactive period to test device availability
                    #print "Exiting idle"
                except:
                    rospy.loginfo('Exception in idle_loop: %s'%sys.exc_info()[0])
                finally:
                    self.mutex.acquire()

            self.diagnostics(2)
        self.mutex.release()

    def init_vars(self):
        self.num_channels = 10
        self.builtinsounds = {}
        self.filesounds = {}
        self.voicesounds = {}
        self.hotlist = []
        if not self.initialized:
            rospy.loginfo('cordial_sound node is ready to play sound')

    def sleep(self, duration):
        try:
            rospy.sleep(duration)
        except rospy.exceptions.ROSInterruptException:
            pass

    def idle_loop(self):
        self.last_activity_time = rospy.get_time()
        while (rospy.get_time() - self.last_activity_time < 10 or
                 len(self.builtinsounds) + len(self.voicesounds) + len(self.filesounds) > 0) \
                and not rospy.is_shutdown():
            #print "idle_loop"
            self.diagnostics(0)
            self.sleep(1)
            self.cleanup()
        #print "idle_exiting"

if __name__ == '__main__':
    soundplay()

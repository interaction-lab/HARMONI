#!/usr/bin/python

#------------------------------------------------------------------------------
# Example tablet use with CoRDial
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

#your package manifest here
import roslib; roslib.load_manifest('cordial_example')

# you'll definitely need to import the following:
import rospy
from cordial_tablet.msg import *
from cordial_tablet.srv import *

# additional imports for this example
import random
import math

class TabletGame:
    def __init__(self):
        #publisher for changes to tokens
        self._token_pub = rospy.Publisher('/CoRDial/tablet/tokens', ChangeToken, queue_size=10)

        #publisher for displaying banners
        self._banner_pub = rospy.Publisher('/CoRDial/tablet/banner', DispBanner, queue_size=10)    
        
        #publisher for reset messages (make the page reload)
        self._refresh_pub = rospy.Publisher('/CoRDial/tablet/reload', Reload, queue_size=1)

        #service that sets up initial game state
        self._setup_srv = rospy.Service('/CoRDial/tablet/setup', Setup, self.setup_cb)

        #subscriber for every time the user "drops" a token
        self._token_sub = rospy.Subscriber('/CoRDial/tablet/tokens/location', TokenLocation ,self.token_change_cb)

    
    def spin(self):
        rospy.spin()

    def token_change_cb(self, msg):
        pass

    def setup_cb(self, req):
        rospy.loginfo("Got setup request from instance named {} with width {} and height {}".format(req.id, req.window_w, req.window_h))
        
        window_width = req.window_w
        window_height = req.window_h

        self.w = window_width
        self.h = window_height

        
        area1 = AddRemoveArea(id = req.id, #what screen should this go on? 
                              name = "box1", #give the area a name
                              text = "Box 1", #text label that will show on the box
                              x = int(window_width/2), #x position of area upper left corner
                              y = 10, #y position of area upper left corner
                              width = int(window_width/3), #width of area
                              height = int(window_height/3), #height of area
                              drawn = True, #should the bounding box and text be shown?
                              arrange = "tile", #should tokens be organized when dropped into area
                              textx = 10, #text label x offset from upper left corner
                              texty = 10, #text label y offset from upper left corner
                              filled=False,
                              img="sanded_plate.png",
                              xpadding = 25,
                              ypadding = 50) #offset from edges of box for organizing tokens

        area2 = AddRemoveArea(id = req.id, name = "box2", text = "Box 2", x = int(window_width/2), y = int(window_height/2), width = int(window_width/3), height = int(window_height/3), drawn = True, arrange = "center", textx = 10, texty = 10, xpadding = 25,ypadding=50,filled=True,bcolor=0xFFCC00)
        
        area3 = AddRemoveArea(id = req.id, name = "box3", text = "Box 3", x = int(window_width/8), y = int(window_height/2), width = int(window_width/3), height = int(window_height/3), drawn = True, arrange = "center", textx = 10, texty = 10, xpadding = 25,ypadding=50,filled=True,bcolor=0x33CC33)


        token1 = ChangeToken(id = req.id, #what screen should this go on? 
                             name = "token1", #give the token a name
                             action = "add;scale", #add the token then resize it
                             img_loc = "circle.png", # must be in cordial_tablet/web/img
                             type = "interactive", #interactive = can be moved, button = can't be moved but can be pressed, anything else = completely static
                             text = "One", #text to be layered on token; mostly useful for debugging; leave out or send empty string for no text
                             x = 150+int((window_width/2-150)/3), #x position of token CENTER
                             y = 100, #y position of token CENTER
                             scalex = 0.5, #multiply x size by how much. must be >0
                             scaley = 0.5) #multiply y size by how much. must be >0 
        #tint = 0xFFFFFF (for example) could be included, but you'd have to put "tint" in the action list.  Tints the image by a certain amount
        token2 = ChangeToken(id=req.id, name="token2", action="add;scale",img_loc="circle.png", type="interactive", text="Two", x = 150+int(2*(window_width/2-150)/3), y=100, scalex=0.5, scaley=0.5)

        reset_button = ChangeToken(id=req.id, name="reset_button", action="add;scale", img_loc="button.png", type="button", text="Reset", x=100, y=100, scalex=0.5, scaley=0.5)

        



        resp = SetupResponse(tokens = [token1,token2,reset_button], 
                             areas = [area1,area2,area3], 
                             background="dots.png") #image must be in cordial_tablet/web/bkgd
        #if the background image is smaller than the window size, it will be tiled


        return resp

    def token_change_cb(self, msg):
        rospy.loginfo("Token {} from screen {} is in areas {}, with x={} and y={}".format(msg.tokenid, msg.instanceid, str(msg.areas), msg.x, msg.y))

        # we'll make some decisions here about what the game should do depending on
        # what tokens have been moved where
        # actions that can be taken on tokens:
        #        add, delete, retexture, move, disable, enable, tint, scale


        if msg.tokenid == "reset_button": #area isn't important for buttons
            # This is the same ChangeToken message from the setup cb
            
            self._token_pub.publish(ChangeToken(id=msg.instanceid,
                                                name="token1",
                                                action="move;retexture;scale;tint;rotate", #>1 action okay
                                                tint=0xFFFFFF,
                                                scalex=0.5,
                                                scaley=0.5,
                                                img_loc="circle.png",
                                                x= 150+int((self.w/2-150)/3),
                                                y= 100,
                                                angle=0))
            self._token_pub.publish(ChangeToken(id=msg.instanceid,
                                                name="token2",
                                                action="move;retexture;scale;tint;rotate",
                                                scalex=0.5,
                                                scaley=0.5,
                                                tint=0xFFFFFF,
                                                img_loc="circle.png",
                                                x= 150+int(2*(self.w/2-150)/3),
                                                y= 100,
                                                angle=0))

        else: # must be one of the non-button tokens
            # box1 randomly tints the token
            if "box1" in msg.areas:
                tint_options = [0xFF0000, 0x00FF00, 0x0000FF]
                # you don't need to include parameters that aren't important for
                # the action you're trying to do.  In this case, we just want to
                # tint the token, so we include tint, but not e.g., x and y
                self._token_pub.publish(ChangeToken(id=msg.instanceid,
                                                    name=msg.tokenid,
                                                    action="tint",
                                                    tint = random.choice(tint_options)))

            # box2 randomly changes the shape of the token
            if "box2" in msg.areas:
                shape_options = ["circle.png", "triangle.png", "plate.png", "star.png"]
                self._token_pub.publish(ChangeToken(id=msg.instanceid,
                                                    name=msg.tokenid,
                                                    action="retexture;scale", # >1 action okay
                                                    img_loc=random.choice(shape_options),
                                                    scalex = 0.5,
                                                    scaley = 0.5))
                
        
            # box3 randomly changes the orientation of the token
            if "box3" in msg.areas:
                angle_options = [0,math.pi/4,math.pi/2,3*math.pi/4,
                                 math.pi,5*math.pi/4, 3*math.pi/2]
                self._token_pub.publish(ChangeToken(id=msg.instanceid,
                                                    name=msg.tokenid,
                                                    action="rotate", # >1 action okay
                                                    angle=random.choice(angle_options)))
                


if __name__=="__main__":
    rospy.init_node("tablet_only_example")
    t = TabletGame()

    t.spin()

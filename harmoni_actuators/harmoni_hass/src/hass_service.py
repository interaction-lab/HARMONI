#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State, ActuatorNameSpace
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from std_msgs.msg import String, Bool
import numpy as np
import json
import requests
#import ast

class HassService(HarmoniServiceManager):
    """
    Hass service
    """

    def __init__(self, name, param):
        """ Initialization of variables and home assistant parameters """
        super().__init__(name)
        self.name = name
        self.service_id = hf.get_child_id(self.name)

        # The Home Assistant uri set in the configuration file
        self.hass_uri = param["hass_uri"]

        # The authorization token (from Home Assistant's settings) set in the configuration file
        self.token = param["token"]

        self.state = State.INIT
        return

    def request(self, data):
        """Complete the request received

        Args:
            data (str): string of json which contains 3 items: {"action": str, "entity": str, "type": str} ....
                action: the action you want to send to home assistant (e.g. turn_on, turn_off)
                type: the entity type of the device (e.g., media_player, switch, light)
                entity: the device on which to do the action (e.g. googlehome8554)

        Returns:
            object: It containes information about the response received (bool) and response message (str)
                response: bool
                message: str
        """

        rospy.loginfo("Start the %s request" % self.name)
        self.state = State.REQUEST

        try:
            # Parse request
            rospy.loginfo("Request: %s " % data)
            json_data= json.loads(data)
            rospy.loginfo("Action: %s " % json_data["action"])
            rospy.loginfo("Entity type: %s " % json_data["type"])
            rospy.loginfo("Entity: %s " % json_data["entity"])

            # POST request
            myJson= {"entity_id": json_data["type"] + "." + json_data["entity"]}
            myHeaders = {"Authorization": "Bearer "+ self.token}

            url = self.hass_uri + 'api/services/' + json_data["type"] +'/' + json_data["action"]
            
            hass_response = requests.post(
                url,
                json=myJson,
                headers=myHeaders
                )

            rospy.loginfo(f"The status code for Home Assistant's response is {hass_response.status_code}")
            # rospy.loginfo(f"Home assistant request url: {hass_response.request.url}")
            # rospy.loginfo(f"Home assistant request headers: {hass_response.request.headers}")
            # rospy.loginfo(f"Home assistant request body: {hass_response.request.body}")

            if hass_response.status_code == 200:
                self.state = State.SUCCESS
                self.response_received = True
                
            else:
                self.start = State.FAILED
                rospy.loginfo("Service call failed")
                rospy.loginfo(f"Home Assistant's response is {hass_response.text}, with status code {hass_response.status_code}")
                rospy.loginfo("Did you put the correct uri and token in the configuration file?")
                self.response_received = True

            self.result_msg = hass_response.text

        except rospy.ServiceException:
            self.start = State.FAILED
            rospy.loginfo("Service call failed")
            self.response_received = True
            self.result_msg = ""

        return {"response": self.state, "message": self.result_msg}



def main():
    """Set names, collect params, and give service to server"""

    service_name = ActuatorNameSpace.hass.name
    instance_id = rospy.get_param("/instance_id")
    service_id = f"{service_name}_{instance_id}"
    try:
        rospy.init_node(service_name)
        params = rospy.get_param(service_name + "/" + instance_id + "_param/")
        s = HassService(service_name, params)
        service_server = HarmoniServiceServer(service_id, s)
        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

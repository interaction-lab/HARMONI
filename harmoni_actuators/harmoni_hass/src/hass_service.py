#!/usr/bin/env python3

# Common Imports
from typing_extensions import OrderedDict
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
from datetime import datetime, timedelta
import pytz
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

        # The path where the authorization token (from Home Assistant's settings) is
        self.credential_path = param["credential_path"]
        with open(self.credential_path) as f:
            d = json.load(f)
            self.token = d["token"]
        
        # POST ACTIONS
        self.post_actions = { "turn_on", "turn_off", "play_media" }

        # Pretend that an appliance has been on for a few hours
        self.simulation = param["simulation"]

        self.state = State.INIT
        return

    def request(self, data):
        """Completes the home assistant request if the input data is json, otherwise forwards the input data.

        Args:
            data (str):  if it contains "{", the string is interpreted as json.
            This string of json which contains 3 items: {"action": str, "entity": str, "type": str} ....
                action: the action you want to send to home assistant (e.g. turn_on, turn_off or check_log)
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

            if "{" not in data:
                rospy.loginfo("No { in data, forwarding message")
                self.result_msg = data
                self.state = State.SUCCESS
                self.response_received = True

            else:
                rospy.loginfo("{ in data")
                data_list = data.split("{")
                message_to_forward = data_list[0]
                data_list[1] = "{"+ data_list[1]
                rospy.loginfo(data_list[0])
                rospy.loginfo(data_list[1])
                # TODO MANAGE MULTIPLE COMMANDS or ONLY COMMANDS NO TEXT

                rospy.loginfo("Request: %s " % data_list[1])
                json_data= json.loads(data_list[1])

                # Responses from bot may have json commands in them but they can also be ignored using this parameter
                if("answer" in json_data and json_data["answer"] == "no"):  
                    self.state = State.SUCCESS
                    self.response_received = True
                    self.result_msg = message_to_forward # No action done

                else:            
                    rospy.loginfo("Action: %s " % json_data["action"])

                    
                    if(json_data["action"] == "check_log"):
                        hass_response = self.check_log(json_data)

                    elif(json_data["action"] in self.post_actions):    
                        hass_response = self.post(json_data)
                        self.result_msg = message_to_forward + self.result_msg


                    rospy.loginfo(f"The status code for Home Assistant's response is {hass_response.status_code}")
                    rospy.loginfo(f"Home assistant request text: {hass_response.text}") 
                    rospy.loginfo(f"Home assistant request url: {hass_response.request.url}")
                    rospy.loginfo(f"Home assistant request headers: {hass_response.request.headers}")
                    rospy.loginfo(f"Home assistant request body: {hass_response.request.body}")           
            
                    if hass_response is not None and hass_response.status_code == 200:
                        self.state = State.SUCCESS
                        self.response_received = True

                    else:
                        self.start = State.FAILED
                        rospy.loginfo("Service call failed")
                        rospy.loginfo(f"Home Assistant's response is {hass_response.text}, with status code {hass_response.status_code}")
                        rospy.loginfo("Did you put the correct uri and token in the configuration file?")
                        self.response_received = True

        except rospy.ServiceException as e:
            self.start = State.FAILED
            rospy.logerr("Service call failed")
            self.response_received = True
            self.result_msg = e

        return {"response": self.state, "message": self.result_msg}


    def check_log(self, json_data):
        """Check if an appliance has been on for some time

        Args:
            data (str): string of json which contains 3 items: {"action": str, "entity": str, "type": str} ....
                action: "check_log"
                entity_id: the entity type of the device (e.g., media_player, switch, light) and the device name (e.g. googlehome8554)

        Returns:
            hass_response (str): It containes the response to the API request api/logbook
        """

        rospy.loginfo("Entity id: %s " + json_data["entity_id"])
        myHeaders = {"Authorization": "Bearer "+ self.token}

        # Home Assistant returns all info in UTC
        dateTimeObj = datetime.now(pytz.utc)
        rospy.loginfo("Current time: %s " % str(dateTimeObj))

        # How much time before the current time I want to check for events (default is 3 hours)
        m = s = d = 0
        h = 3

        if("hours" in json_data):
            h = int(json_data["hours"])
        if("days" in json_data):
            d = int(json_data["days"])
        if("minutes" in json_data):
            m = int(json_data["minutes"])
        if("seconds" in json_data):
            s = int(json_data["seconds"])

        delta = timedelta(
            days = d,
            hours = h,
            minutes = m,
            seconds = s
            )

        rospy.loginfo("Timespan to check: %s " % str(delta))
        timeToCheck = dateTimeObj - delta

        # formatting "2021-05-24T10:00:00+00:00"       
        timeToCheckFormatted = str(timeToCheck.year) + "-" + str(timeToCheck.month)  + "-" + str(timeToCheck.day) + "T" + str(timeToCheck.hour) +":"+ str(timeToCheck.minute) + ":" + str(timeToCheck.second) + "+00:00"
        rospy.loginfo("Time to check: %s " % timeToCheckFormatted)

        url = self.hass_uri + 'api/logbook/' + timeToCheckFormatted +'?' + "entity:"+ json_data["entity_id"]
        
        hass_response = requests.get(
            url,
            headers=myHeaders
            )
        
        json_array = hass_response.json()
        eventTime = ""

        for item in json_array:
            if "context_service" in item: 

                if item["context_service"] in self.post_actions:
                    eventTime = item["when"]

                elif item["context_service"] in self.post_actions:
                    eventTime = ""

            # TODO ALSO CHECK STATE = "OFF" IF ENTITY_ID IS THE CORRECT ONE

        alertUser = False
        
        if eventTime is not "":
            dateEventTime = datetime.strptime(eventTime, '%Y-%m-%dT%H:%M:%S.%f+00:00')
            dateEventTime = dateEventTime.replace(tzinfo=pytz.utc)
            delta = dateTimeObj - dateEventTime
            rospy.loginfo(f"Timespan appliance on: {str(delta)}")

            # Check if the home appliance has been on for a few hours
            if delta > timedelta(hours=2):
                alertUser = True

# TODO custom return msg, saved outside of this code

            if alertUser == True:
                self.result_msg = "LOG: oven still on"
            else:
                self.result_msg = "NONE"
            rospy.loginfo(f"Is appliance on? {alertUser}")

        # Check if this is a simulation
        if self.simulation == True:
            self.result_msg = "LOG: oven still on"
        rospy.loginfo(f"Is simulation on? {self.simulation}")
        
        return hass_response


    def post(self, json_data):
        """ Do an API POST call

        Args:
            data (str): string of json which contains 2 or 3 items: {"action": str, "entity_id": str} ....
                action: "turn_on" or "turn_off" or other "post" actions
                entity_id: the entity type of the device (e.g., media_player, switch, light)
                           with, separated by a dot, the name of the device on which to do the action (e.g. googlehome8554)
                EXAMPLE "entity_id" : "mediaplayer.googlehome8554"

        Returns:
            hass_response (str): It containes the response to the API request api/services
        """

        if "entity_id" in json_data:
            rospy.loginfo("Entity: %s " % json_data["entity_id"])

        parameters = {}
        for x in json_data:
            if x != "answer" and x != "action": 
                    parameters[x] = json_data[x]
        rospy.loginfo("json " + str(parameters))

        myHeaders = {"Authorization": "Bearer "+ self.token}

        type = json_data["entity_id"].split(".")[0]

        url = self.hass_uri + 'api/services/' + type +'/' + json_data["action"]
        
        hass_response = requests.post(
            url,
            json=parameters,
            headers=myHeaders,

            # SELF-SIGNED CERTIFICATE FOR A LOCAL CONNECTION
            verify=False
            )

        # self.result_msg = hass_response.text
        self.result_msg = " Fatto"

        return hass_response


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

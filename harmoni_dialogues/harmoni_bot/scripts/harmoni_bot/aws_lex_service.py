#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import DialogueNameSpace
import boto3


class AWSLexService(HarmoniServiceManager):
    """
    Amazon Lex service
    """

    def __init__(self, name, param):
        super().__init__(name)
        """ Initialization of variables and lex parameters """
        self.user_id = param["user_id"]
        self.bot_name = param["bot_name"]
        self.bot_alias = param["bot_alias"]
        self.region_name = param["region_name"]
        """ Setup the lex request """
        self.setup_aws_lex()
        """Setup the lex service as server """
        self.state = State.INIT
        return

    def setup_aws_lex(self):
        self.lex_client = boto3.client("lex-runtime", region_name=self.region_name)
        return

    def request(self, input_text):
        rospy.loginfo("Start the %s request" % self.name)
        self.state = State.REQUEST
        textdata = input_text
        try:
            lex_response = self.lex_client.post_content(
                botName=self.bot_name,
                botAlias=self.bot_alias,
                userId=self.user_id,
                contentType="text/plain; charset=utf-8",
                accept="text/plain; charset=utf-8",
                inputStream=textdata,
            )
            self.state = State.SUCCESS
            if "intentName" in lex_response:
                if lex_response["dialogState"] == "Fulfilled":
                    rospy.loginfo("The dialogue is fulfilled, end the conversation.")
            rospy.loginfo("The response is %s" % (lex_response["message"]))
            self.response_received = True
            self.result_msg = lex_response["message"]
        except rospy.ServiceException:
            self.start = State.FAILED
            rospy.loginfo("Service call failed")
            self.response_received = True
            self.result_msg = ""
        return


def main():
    service_name = DialogueNameSpace.bot.name
    name = rospy.get_param("/name_" + service_name + "/")
    test = rospy.get_param("/test_" + service_name + "/")
    test_input = rospy.get_param("/test_input_" + service_name + "/")
    test_id = rospy.get_param("/test_id_" + service_name + "/")
    try:
        rospy.init_node(service_name)
        list_service_names = hf.get_child_list(service_name)
        service_server_list = []
        last_event = ""  # TODO
        for service in list_service_names:
            rospy.loginfo(service)
            service_id = hf.get_child_id(service)
            param = rospy.get_param(name + "/" + service_id + "_param/")
            print(param)
            s = AWSLexService(service, param)
            service_server_list.append(
                HarmoniServiceServer(name=service, service_manager=s)
            )
            if test and (service_id == test_id):
                rospy.loginfo("Testing the %s" % (service))
                s.request(test_input)
        if not test:
            for server in service_server_list:
                server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

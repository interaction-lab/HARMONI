#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
import boto3
import sys
from harmoni_common_lib.constants import State, RouterDialogue, HelperFunctions
from harmoni_common_lib.child import WebServiceServer
from harmoni_common_lib.service_manager import HarmoniExternalServiceManager

class AWSLexService(HarmoniExternalServiceManager):
    """
    Amazon Lex service
    """

    def __init__(self, name, param):
        """ Initialization of variables and lex parameters """
        rospy.loginfo("AWS Lex initializing")
        self.name = name
        self.user_id = param["user_id"]
        self.bot_name = param["bot_name"]
        self.bot_alias = param["bot_alias"]
        self.region_name = param["region_name"]
        """ Setup the lex request """
        self.setup_aws_lex()
        """Setup the lex service as server """
        self.state = State.INIT 
        super().__init__(self.state)
        return

    def setup_aws_lex(self):
        self.lex_client = boto3.client('lex-runtime', region_name=self.region_name)
        return

    def response_update(self, response_received, state, result_msg):
        super().update(response_received=response_received, state = state, result_msg=result_msg)
        return

    def test(self):
        super().test()
        rospy.loginfo("Test the %s service" % self.name)
        success = True
        return success

    def request(self, input_text):
        rospy.loginfo("Start the %s request" % self.name)
        self.state = State.REQUEST
        rate = "" #TODO: TBD
        super().request(rate)
        textdata = input_text
        try:
            lex_response = self.lex_client.post_content(botName = self.bot_name,
														botAlias = self.bot_alias,
														userId = self.user_id,
														contentType = 'text/plain; charset=utf-8',
														accept = 'text/plain; charset=utf-8',
														inputStream = textdata)
            self.state = State.SUCCESS
            rospy.loginfo("The response is %s" %(lex_response["message"]))
            self.response_update(response_received=True, state=self.state, result_msg=lex_response["message"])
        except rospy.ServiceException:
            self.start = State.FAILED
            rospy.loginfo("Service call failed")
            self.response_update(response_received=True, state=self.state, result_msg="")
        return


def main():
    args = sys.argv
    try:
        service_name = RouterDialogue.LEX.value
        rospy.init_node(service_name + "_node")
        list_service_names = HelperFunctions.get_child_list(service_name)
        service_server_list = []
        last_event = ""  # TODO
        for service in list_service_names:
            print(service)
            service_id = HelperFunctions.get_child_id(service)
            param = rospy.get_param("/"+service_id+"_param/")
            s = AWSLexService(service, param)
            service_server_list.append(WebServiceServer(name=service, service_manager=s))
            if args[1] and (service_id == args[3]):
                rospy.loginfo("Testing the %s" %(service))
                s.request(args[2])
        if not args[1]:
            for server in service_server_list:
                server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rospy
import requests
from requests.exceptions import Timeout
import socket
import sys
from subprocess import check_output


def is_int(value):
    try:
        int(value)
        return True
    except ValueError:
        return False

#DIR_IDX = 1
PORT_IDX = 2

if __name__ == "__main__":
    rospy.init_node("misty_caller")
    robot_ip = rospy.get_param("/robot_ip")

    local_ip = str(check_output(['hostname', '--all-ip-addresses']).split()[0])[2:-1]

    args = sys.argv
    if is_int(args[PORT_IDX]):
        port = args[PORT_IDX]

    payload = {'URL': 'http://' + local_ip + ':' + port ,
                'layer': 'DefaultWebViewLayer1'}
    print(payload['URL'])

    try:
        response = requests.post('http://{}/api/webviews/display'.format(robot_ip), 
                                    params = payload,
                                   timeout = 4)
    except Timeout:
        rospy.logwarn("web_player failed: The ip of the robot appears unreachable")
    while not rospy.is_shutdown():
        rospy.sleep(1)
    #rospy.spin()
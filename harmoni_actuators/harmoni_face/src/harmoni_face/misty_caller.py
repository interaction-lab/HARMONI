import rospy
import requests
from requests.exceptions import Timeout
import socket
import sys


#DIR_IDX = 1
PORT_IDX = 2
if __name__ == "__main__":
    rospy.init_node("misty_caller")
    robot_ip = rospy.get_param("/robot_ip")
    
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    if is_int(args[PORT_IDX]):
        port = args[PORT_IDX]

    payload = {'URL': 'http://' + local_ip + '/' + port}

    try:
        response = requests.post('http://{}/api/webviews/display'.format(self.robot_ip), 
                                    params = payload,
                                   timeout = 1)
    except Timeout:
        rospy.logwarn("web_player failed: The ip of the robot appears unreachable")
    while not rospy.is_shutdown():
        rospy.sleep(1)
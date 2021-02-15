#!/usr/bin/env python3

import rospy
import os
from std_srvs.srv import Trigger

if __name__ == "__main__":
    rospy.sleep(3)
    rospy.wait_for_service("start_browser")
    service_call = rospy.ServiceProxy("start_browser", Trigger)
    try:
        response = service_call()
        print(response)
    except Exception:
        print("Service call failed")
        os.system("pkill init")

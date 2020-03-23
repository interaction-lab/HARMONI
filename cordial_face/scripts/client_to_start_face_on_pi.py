#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger

if __name__ == "__main__":
	rospy.sleep(3)
	rospy.wait_for_service('start_pi_browser')
	service_call = rospy.ServiceProxy('start_pi_browser', Trigger)
	try:
		response = service_call()
		print(response)
	except rospy.ServiceException, e:
		print ("Service call failed: %s" % e)

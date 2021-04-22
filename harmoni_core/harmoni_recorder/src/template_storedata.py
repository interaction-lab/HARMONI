#!/usr/bin/env python3

# Common Imports
import rospy
import roslib
from mongodb_client import MongoDBClient

def main():
    try:
        name="mongodb_client"
        rospy.init_node(name)
        mdb_client = MongoDBClient(client_uri="mongodb://172.18.3.3:27017/", username="root", password="example")
        client = mdb_client.get_client()
        client.harmoni.users.insert_one({"name": "Chris"})
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
#!/usr/bin/env python3

# Common Imports
import rospy
import roslib
import pymongo  # package for working with MongoDB


class MongoDBClient(object):

    def __init__(self, client_uri, username, password):
        self.client = self.mongodb_connect(client_uri, username, password)

    def mongodb_connect(self, client_uri,username, password):
        try:
            client = pymongo.MongoClient(client_uri,
                        username=username,
                        password=password)
            return client
        except pymongo.errors.ConnectionFailure:
            print("Failed to connect to server {}".format(client_uri))

    def get_client(self):
        return self.client


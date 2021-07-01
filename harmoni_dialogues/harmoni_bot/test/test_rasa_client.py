#!/usr/bin/env python3
import mock
import requests
import unittest

from harmoni_bot.rasa_client import RasaClient


class TestRasaClient(unittest.TestCase):

    def setUp(self):
        self.client = RasaClient("localhost", 5005)

    @mock.patch("requests.post")
    def test_get_rasa_response(self, mock_post):
        recipient_id = "test_user"
        text = "Welcome to the Interaction Lab!"



if __name__ == "__main__":
    unittest.main()

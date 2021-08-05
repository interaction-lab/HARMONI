#!/usr/bin/env python3
import mock
import requests
import unittest

from harmoni_bot.rasa_client import RasaClient


class TestRasaClient(unittest.TestCase):

    def setUp(self):
        self.client = RasaClient("localhost", 5005)

    @mock.patch("harmoni_bot.rasa_client.requests.post")
    def test_get_rasa_response(self, mock_post):
        recipient_id = "test_user"
        text = "Welcome to the Interaction Lab!"
        mock_post.return_value.json.return_value = [
            {"recipient_id": recipient_id, "text": text}
        ]
        rasa_response = self.client.get_rasa_response("Hello world!")
        assert rasa_response == text


if __name__ == "__main__":
    unittest.main()

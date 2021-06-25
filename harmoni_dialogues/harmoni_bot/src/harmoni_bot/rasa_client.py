#!/usr/bin/env python3
import logging
import requests

logging.basicConfig(level=logging.INFO)


class RasaClient:

    def __init__(
            self,
            host="localhost",
            port=5005
    ):
        self._host = host
        self._port = port

    def get_rasa_response(self, input_text):
        headers = {
            'Content-Type': 'application/json',
        }
        data = '{ "sender": "test_user", "message": "' + input_text + '", "metadata": {} }'
        response = requests.post(f"http://{self._host}:{self._port}/webhooks/myio/webhook", headers=headers, data=data)
        logging.info(f"Bot response: \n{response.text}\n")
        try:
            text = response.json()[0]["text"]
        except IndexError:
            logging.error("Warning: Bad index")
            text = "I didn't catch that"
        return text


if __name__ == "__main__":
    rasa_client = RasaClient()
    r = "Let's start a conversation with the rasa bot. What would you like to say?\n->"
    while True:
        i = input(r)
        r = rasa_client.get_rasa_response(i) + "\n->"

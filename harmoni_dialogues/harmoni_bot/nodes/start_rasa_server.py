#!/usr/bin/env python3

import os
import rasa
import rospy


def _start_rasa_server(rasa_assistant_path, host, port):
    config = os.path.join(rasa_assistant_path, "config.yml")
    connector = "addons.custom_channel.MyIO"
    credentials = os.path.abspath(os.path.join(rasa_assistant_path, "credentials.yml"))
    data = os.path.abspath(os.path.join(rasa_assistant_path, "data"))
    domain = os.path.join(rasa_assistant_path, "domain.yml")
    endpoints = os.path.join(rasa_assistant_path, "endpoints.yml")
    model = os.path.join(rasa_assistant_path, "models")

    rasa.train(
        domain=domain,
        config=config,
        training_files=data,
        output=model
    )

    rasa.run(
        model=model,
        endpoints=endpoints,
        connector=connector,
        credentials=credentials,
        **{"host": host, "port": port}
    )

def main():
    instance_id = rospy.get_param("instance_id")  # "default"
    params = rospy.get_param("rasa" + "/" + instance_id + "_param/")

    host = params["host"]
    port = params["port"]
    rasa_assistant = params["rasa_assistant"]
    harmoni_bot_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if rasa_assistant == "rasa_example" or rasa_assistant == "rasa_greeter":
        bot_path = f"src/{rasa_assistant}"
    else:
        rospy.logerr("Not a valid Rasa bot, defaulting to rasa_example")
        bot_path = "src/rasa_example"
    rasa_assistant_path = os.path.abspath(os.path.join(
        harmoni_bot_dir, bot_path
    ))

    _start_rasa_server(rasa_assistant_path, host, port)


if __name__ == "__main__":
    main()

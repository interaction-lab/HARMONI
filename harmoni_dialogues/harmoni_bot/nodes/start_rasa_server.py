#!/usr/bin/env python3

import argparse
import os

from rasa.run import run
from rasa.train import train


if __name__ == "__main__":
    config = "/root/harmoni_catkin_ws/src/HARMONI/harmoni_dialogues/harmoni_bot/src/rasa_example/config.yml"
    data = "/root/harmoni_catkin_ws/src/HARMONI/harmoni_dialogues/harmoni_bot/src/rasa_example/data"
    domain = "/root/harmoni_catkin_ws/src/HARMONI/harmoni_dialogues/harmoni_bot/src/rasa_example/domain.yml"
    endpoints = "/root/harmoni_catkin_ws/src/HARMONI/harmoni_dialogues/harmoni_bot/src/rasa_example/endpoints.yml"
    model = "/root/harmoni_catkin_ws/src/HARMONI/harmoni_dialogues/harmoni_bot/src/rasa_example/models"

    # parser = argparse.ArgumentParser()
    # parser.add_argument("--config")
    # parser.add_argument("--data")
    # parser.add_argument("--domain")
    # parser.add_argument("--model")
    # args = parser.parse_args()

    # train(
    #     domain=args.domain,
    #     config=args.config,
    #     training_files=args.data,
    #     output=args.model
    # )
    train(
        domain=domain,
        config=config,
        training_files=data,
        output=model
    )

    connector = "addons.custom_channel.MyIO"
    credentials = "/root/harmoni_catkin_ws/src/HARMONI/harmoni_dialogues/harmoni_bot/src/rasa_example/credentials.yml"

    # run(
    #     model=args.model,
    #     endpoints=args.endpoints,
    #     connector=args.connector,
    #     credentials=args.credentials,
    #     **{"host": "localhost", "port": 5005}
    # )
    run(
        model=model,
        endpoints=endpoints,
        connector=connector,
        credentials=credentials,
        **{"host": "localhost", "port": 5005}
    )

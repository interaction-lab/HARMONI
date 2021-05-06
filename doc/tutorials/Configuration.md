# Configuration

As described before, Harmoni is organized as follows:
  - harmoni_actuators - _controlling hardware (e.g. motors, screens, speakers)_
  - harmoni_core
    - harmoni_common_lib - _defines the Harmoni unit and helper functions_
    - harmoni_common_msgs - _defines harmoni messages and actions_
    - harmoni_decision - _highest level controller, can play patterns or individual units_
    - harmoni_pattern - _middle level decision player, defines sequences or patterns for dialog_
    - harmoni_recorder - _records interactions (WIP)_
  - harmoni_detectors - _extracting useful information from sensor signals (e.g. transcriptions, facial locations, etc.)_
  - harmoni_dialogues - _processing user speech (text) and return robot speech (text)_
  - harmoni_sensors - _reading physical sensors and publishing sensor streams_

## Startup
Throughout Harmoni, there are some important configuration points. At the highest level of configuration is the harmoni_decision module. In this module the configuration.yaml specifies the run configuration of nodes that the system will run. The configuration is structured as:

launch_group:
  node: ["name1", "name2", "etc"]

These nodes are launched withe launcher.launch file the launcher.py, which will set up the desired services. Also within this package are the decision manager scripts, which can be used to control and play patterns.

Beneath the decision module we configure the harmoni_pattern, which plays a sequence of nodes just like a series of notes on an instrument. What pattern to use is configured in the sequence.launch file, while the default parameters for the player are configured in the configuration.yaml. 

## Running
Once the system configured for proper setup, we must consider what we actually want the system to do. The patterns of steps we wish to run are described in the json files in the pattern_scripting folder of the harmoni_pattern module. [Please see here for a full description of the patterns jsons]().

We can also launch difference modules directly using the 'test' launch param.

## Dialoging
Dialoging is configured with support for google and amazon chatbots. The bot and project configuration are described in the configuration.yaml, and which of these is used is configured during launch. [Describe where the bot choice is set]

Bot's are expected to take and return text. The initial trigger for the bot (if the bot is expected to speak first) is configured in the pattern step description. For example:

 "bot_default": {
 "action_goal": "REQUEST",
 "resource_type": "service",
 "wait_for": "new",
}

[Describe config for lex and google bots]

## Hardware
Microphone set with .asoundrc

## Docker

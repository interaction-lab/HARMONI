# ROS API

_How are the various services communicated with in HARMONI?_

HARMONI is made up of a number of services; most services implement this API. The ROS API was briefly referenced in the [architecture overview](overview/Architecture), but we go into depth here.

Action servers and topics which are part of the ROS API all reside within their specific namespace, e.g. `/harmoni/detecting/face_detect/` for the face detector and `/harmoni/actuating/speaker/` for the speaker.

## Action Servers

With a few exceptions, each action server supports actions (goal/feedback/result) with the following action types. OFF and ON are always supported.

- OFF = 0 -- Turn off the service (may be useful for saving compute power; most services initialize to this state)
- ON = 1 -- Turn on the service (this is often the first thing you need to send in order to use a service)
- PAUSE = 2 -- Pause the service (exact meaning depends on service).
- REQUEST = 3 -- Ask the service for something (often you will also include data with your request)
- DO = 4 -- Ask the service to do something in particular (Often used by actuators)

_Note: If an action type is not supported by a service and you request it, you should receive an error message._

## Topics

Some services publish and subscribe to topics to handle continuous dataflow (especially sensors and detectors). This follows [ROS design patterns](https://wiki.ros.org/ROS/Patterns/Communication#Communication_via_Topics_vs_Services_vs_X).

TODO: Enter all topics (include publishers and subscribers)
# This files contains your custom actions which can be used to run
# custom Python code.
#
# See this guide on how to implement these action:
# https://rasa.com/docs/rasa/custom-actions


# This is a simple example for a custom action which utters "Hello World!"

# from typing import Any, Text, Dict, List
#
# from rasa_sdk import Action, Tracker
# from rasa_sdk.executor import CollectingDispatcher
#
#
# class ActionHelloWorld(Action):
#
#     def name(self) -> Text:
#         return "action_hello_world"
#
#     def run(self, dispatcher: CollectingDispatcher,
#             tracker: Tracker,
#             domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
#
#         dispatcher.utter_message(text="Hello World!")
#
#         return []
# This files contains your custom actions which can be used to run
# custom Python code.
#
# See this guide on how to implement these action:
# https://rasa.com/docs/rasa/core/actions/#custom-actions/


# This is a simple example for a custom action which utters "Hello World!"

import json
from rasa_sdk.forms import FormAction
from rasa_sdk.events import AllSlotsReset
from rasa_sdk.events import SlotSet
from typing import Any, Text, Dict, List
from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher


class ActionHelloWorld(Action):

    def name(self) -> Text:
        return "action_hello_world"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        dispatcher.utter_message(text="Hello World!")

        return []


class SalesForm(FormAction):
    """Collects sales information and adds it to the spreadsheet"""

    def name(self):
        return "sales_form"

    @staticmethod
    def required_slots(tracker):
        return [
            "job_function",
            "use_case",
            "budget",
            "person_name",
            "company",
            "business_email",
        ]

    def submit(
        self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any],
    ) -> List[Dict]:

        dispatcher.utter_message("Thanks for getting in touch, we’ll contact you soon")
        return []


class CheckIn(Action):

    def name(self):
        return "check_in"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        json_file = open('researchers.json', 'r')
        data = json.load(json_file)
        json_file.close()
        for p in data['Researchers']:
            dispatcher.utter_message(text=str(tracker.get_slot('name')))
            if (p['Name'] == tracker.get_slot('name')):
                p['Present'] = True
                break
        json_file = open('researchers.json', 'w')
        json.dump(data, json_file)
        json_file.close()
        return []


class CheckOut(Action):

    def name(self):
        return "check_out"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        json_file = open('researchers.json', 'r')
        data = json.load(json_file)
        json_file.close()
        for p in data['Researchers']:
            dispatcher.utter_message(text=str(tracker.get_slot('name')))
            if (p['Name'] == tracker.get_slot('name')):
                p['Present'] = False
                break
        json_file = open('researchers.json', 'w')
        json.dump(data, json_file)
        json_file.close()
        return []


class findWorker(Action):

    def name(self):
        return "find_worker"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        json_file = open('researchers.json', 'r')
        data = json.load(json_file)
        json_file.close()
        for p in data['Researchers']:
            dispatcher.utter_message(text=str(tracker.get_slot('findWorkerName')))
            if (p['Name'] == tracker.get_slot('findWorkerName')):
                if p['Present']:
                    dispatcher.utter_message(text="Yes! They are here")
                    return
        dispatcher.utter_message(text="Sorry! They are not here right now")
        return []

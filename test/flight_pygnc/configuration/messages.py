import os
import unittest

from .. import context
from pygnc.configuration.messages import message_port_dict
from pygnc.configuration.tasks import task_list


class TestMessagesConfiguration(unittest.TestCase):
    def test_ports_unique(self):

        # ensure port numbers are unique
        port_set = set()
        port_count = 0
        for message_name, message_config in message_port_dict.items():
            port_set.add(message_config["publisher_port"])
            port_count += 1
            port_set.add(message_config["synchronizer_port"])
            port_count += 1

        self.assertEqual(len(port_set), port_count)

    def test_task_names_match(self):
        task_names = [task["name"] for task in task_list]
        for message_name, message_config in message_port_dict.items():
            for subscriber_task_name in message_config["subscribers"]:
                self.assertIn(subscriber_task_name, task_names)

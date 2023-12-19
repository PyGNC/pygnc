import os
import unittest

from .. import context
from pygnc.configuration.messages import message_port_dict


class TestMessagesConfiguration(unittest.TestCase):
    def test_ports_unique(self):

        # ensure port numbers are unique
        port_set = set()
        port_count = 0
        for message_name, message_config in message_port_dict.items():
            port_set.add(message_config["publisher_port"])
            port_count += 1
            for subscriber in message_config["subscribers"]:
                port_set.add(subscriber["reply_port"])
                port_count += 1

        self.assertEqual(len(port_set), port_count)

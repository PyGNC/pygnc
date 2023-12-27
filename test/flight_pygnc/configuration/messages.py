import os
import unittest

from .. import context
from pygnc.configuration.messages import message_port_dict


class TestMessagesConfiguration(unittest.TestCase):
    def test_ports_unique(self):
        # ensure port numbers are unique
        port_set = set()
        for message_name, message_port in message_port_dict.items():
            port_set.add(message_port)
        self.assertEqual(len(port_set), len(message_port_dict))

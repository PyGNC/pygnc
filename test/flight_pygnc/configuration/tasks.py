import os
import unittest

from .. import context
from pygnc.configuration.tasks import task_list


class TestTasksConfiguration(unittest.TestCase):
    def test_ports_unique(self):

        # ensure port numbers are unique
        port_set = set([task.port for task in task_list])
        self.assertEqual(len(port_set), len(task_list))
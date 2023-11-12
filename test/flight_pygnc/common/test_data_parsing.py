import unittest

from .. import context
from pygnc.common import data_parsing

print(f"test_data_parsing")
data_parsing.data_parsing_foo()


class TestDataParsing(unittest.TestCase):
    def test_foo(self):
        self.assertTrue(True)
        data_parsing.data_parsing_foo()

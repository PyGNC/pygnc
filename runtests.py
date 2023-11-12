"""
This is equivalent to running `python3 -m unittest` in the repository root directory,
but this can be run from anywhere.

To run an individual test file, run from this directory level
`python3 -m unittest test/path/to/test_file.py`
"""

import os
import unittest

thisdir = os.path.abspath(os.path.dirname(__file__))
testdir = os.path.join(thisdir, "test")

tests = unittest.defaultTestLoader.discover(testdir, top_level_dir=thisdir)
testsuite = unittest.TestSuite(tests)
unittest.TextTestRunner().run(testsuite)

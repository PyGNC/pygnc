#!/usr/bin/env python3
"""
This is equivalent to running `python3 -m unittest` in the repository root directory,
but this can be run from anywhere.

To run an individual test file, run from this directory level
`python3 -m unittest test/path/to/test_file.py`

nose2 makes it easy to activate a debugger when tests fail
`pip install nose2`
from ./
`nose2 --debugger`
"""

import os
import unittest

thisdir = os.path.abspath(os.path.dirname(__file__))
testdir = os.path.join(thisdir, "test")

tests = unittest.defaultTestLoader.discover(testdir, top_level_dir=thisdir)
testsuite = unittest.TestSuite(tests)
unittest.TextTestRunner().run(testsuite)

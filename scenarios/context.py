"""
Add pygnc/flight/* to the python path
"""
# fmt: off
import os
import sys
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'flight'))) # noqa
# fmt: on
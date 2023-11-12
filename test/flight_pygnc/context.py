"""
Add flight/pygnc as a module that is importable after this is imported
"""
# fmt: off
import os
import sys
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..', 'flight'))) # noqa
# fmt: on

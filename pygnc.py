"""
pygnc.py

The file to be started by the pycubed controller.
"""
import sys
import os


def main(pygnc_timeout, spacecraft_time):
    pass


if __name__ == "__main__":
    assert len(sys.argv) == 3, "must pass timeout and time.time() ints as arguments"

    # timeout in minutes
    pygnc_timeout = int(sys.argv[-2])
    # spacecraft time
    spacecraft_time = int(sys.argv[-1])

    main(pygnc_timeout, spacecraft_time)

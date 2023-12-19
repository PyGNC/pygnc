"""
pygnc_main.py

The file to be started by the pycubed controller.
The only process that should communicate with the pycubed controller via stdio or tty.
"""
from multiprocessing import Process
import os
import sys
import time

from pygnc.configuration import pygnc as pygnc_config
from pygnc.configuration.tasks import task_list


def start_processes():
    for task in task_list:
        if task["main"] is not None:
            print(f"Starting {task['name']}")
            p = Process(target=task["main"], args=())
            p.start()
            task["process"] = p
        else:
            task["process"] = None


def monitor_processes(timeout):
    all_processes_alive = True
    while all_processes_alive:
        for task in task_list:
            if (task["process"] is not None) and (not task["process"].is_alive()):
                all_processes_alive = False
                print(f"{task['name']} has terminated")


def main(pygnc_timeout, spacecraft_time):
    processes = start_processes()
    monitor_processes(pygnc_timeout)


if __name__ == "__main__":
    assert len(sys.argv) == 3, "must pass timeout and time.time() ints as arguments"

    # timeout in minutes
    pygnc_timeout = int(sys.argv[-2])
    # spacecraft time
    spacecraft_time = int(sys.argv[-1])

    main(pygnc_timeout, spacecraft_time)

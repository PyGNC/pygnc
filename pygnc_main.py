"""
pygnc.py

The file to be started by the pycubed controller.
The only process that should communicate with the pycubed controller via stdio or tty.
"""
from multiprocessing import Process
import os
import sys
import time

import pygnc.tasks.orbit_estimator as orbit_estimator_task


def start_processes():
    processes = dict()
    orbit_estimator_p = Process(target=orbit_estimator_task.main, args=("~/sense.bin",))
    orbit_estimator_p.start()
    processes["orbit_estimator"] = orbit_estimator_p

    return processes


def monitor_processes(processes, timeout):
    all_processes_alive = True
    while all_processes_alive:
        for process_name, process in processes.items():
            if not process.is_alive():
                all_processes_alive = False
                print(f"{process_name} has terminated")


def main(pygnc_timeout, spacecraft_time):
    processes = start_processes()
    monitor_processes(processes, pygnc_timeout)


if __name__ == "__main__":
    assert len(sys.argv) == 3, "must pass timeout and time.time() ints as arguments"

    # timeout in minutes
    pygnc_timeout = int(sys.argv[-2])
    # spacecraft time
    spacecraft_time = int(sys.argv[-1])

    main(pygnc_timeout, spacecraft_time)

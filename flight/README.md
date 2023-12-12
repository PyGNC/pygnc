# Flight Code
Only code that will go on the satellite should be contained in this directory.

The entry point to the flight code is `pygnc_main.py`. This starts tasks in `pygnc/tasks/`. The tasks use algorithms from `pygnc/algorithms`, and common functionality shared between tasks (such as message formats and data parsing) is found in `pygnc/common`.

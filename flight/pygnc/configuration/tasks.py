"""
Defines the tasks for pygnc to run.
"""

from ..tasks import orbit_estimator as orbit_estimator_task

# {"name": "TaskName",
#  "port": ####,
#  "main": task.main
#  }
task_list = [
    {
        "name": "pygnc_main", # name of the task
        "port": 5660, # port this publishes zmq messages on - should be unique
        "main": None, #  entry function for the task (None here because this is the root task)
    },
    {
        "name": "orbit_estimator",
        "port": 5661,
        "main": orbit_estimator_task.main,
    },
]

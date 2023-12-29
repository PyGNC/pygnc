"""
Defines the tasks for pygnc to run.
"""

from ..tasks import orbit_estimator as orbit_estimator_task
from ..tasks import telemetry_logger as telemetry_logger_task
from ..tasks import ranging_estimator as ranging_estimator_task
from ..common import messages

# {"name": "TaskName",
#  "port": ####,
#  "main": task.main
#  }
task_list = [
    {
        "name": "pygnc_main",  # name of the task
        "main": None,  #  entry function for the task (None here because this is the root task)
    },
    {
        "name": "orbit_estimator",
        "main": orbit_estimator_task.main,
    },
    {
        "name": "telemetry_logger",
        "main": telemetry_logger_task.main,
    },
    {
        "name": "ranging_estimator",
        "main": ranging_estimator_task.main,
    }
]

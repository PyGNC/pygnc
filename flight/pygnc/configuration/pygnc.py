import os
import warnings

batch_sensor_gps_filepath = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "..",
        "..",
        "scenarios",
        "default_scenario",
        "batch_sensor_gps_data.bin",
    )
)
warnings.warn(f"using scenario path {batch_sensor_gps_filepath}")

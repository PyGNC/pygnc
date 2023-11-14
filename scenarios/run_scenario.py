#!/usr/bin/env python3

import argparse
import numpy as np
import os

import context
import pygnc_main  # type: ignore 

# full linewidth printing
np.set_printoptions(
    edgeitems=30, linewidth=100000, formatter=dict(float=lambda x: "%.3e" % x)
)  

def main(scenario_path, timeout, spacecraft_time):
    pygnc_main.pygnc_config.batch_sensor_gps_filepath = os.path.join(scenario_path, "batch_sensor_gps_data.bin")
    pygnc_main.main(timeout, spacecraft_time)



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run the pygnc software stack locally with a scenario as input, bypassing the tty")
    parser.add_argument('scenario_path', help="Path to the scenario directory")
    parser.add_argument('--timeout', '-t', default=600, help="Allotted runtime in seconds - pi will shut down after this time")
    parser.add_argument('--spacecraft_time', '-s', default=1699924855, help="Spacecraft time since Unix epoch in seconds")

    args = parser.parse_args()

    scenario_name = os.path.basename(args.scenario_path)
    print(f"Running {scenario_name}")
    main(args.scenario_path, args.timeout, args.spacecraft_time)

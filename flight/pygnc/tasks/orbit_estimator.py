import brahe
import numpy as np
import time


from ..common import data_parsing, transformations
from ..configuration import orbit_estimator as oe_config
from ..algorithms.OrbitEstimator import OrbitEKF


def update_orbit_ekf(orbit_ekf, gps_message, prev_epoch=None):
    # get position and velocity state from gps
    state_measurement_ecef = np.concatenate(
        (gps_message.position, gps_message.velocity)
    )
    measurement_epoch = transformations.gps_week_milliseconds_to_brahe_epoch(
        gps_message.gps_week, gps_message.gps_milliseconds
    )
    state_measurement_eci = brahe.frames.sECEFtoECI(
        measurement_epoch, state_measurement_ecef
    )
    if prev_epoch is None:
        # need to initialize ekf state with first measurement
        orbit_ekf.initialize_state(state_measurement_eci)
    else:
        dt = prev_epoch - measurement_epoch
        orbit_ekf.update(state_measurement_eci, dt)

    return measurement_epoch


def main(batch_gps_sensor_data_filepath):
    print("Orbit Estimator Task")

    # instantiate orbit estimator
    orbit_ekf = OrbitEKF()

    # batch update orbit estimator
    batch_data = data_parsing.unpack_batch_sensor_gps_file_to_messages_iterable(
        batch_gps_sensor_data_filepath
    )
    packet_count = 0
    prev_epoch = None
    for bd in batch_data:
        print(f"Packet count = {packet_count}")
        _, gps_message = bd
        prev_epoch = update_orbit_ekf(orbit_ekf, gps_message, prev_epoch)
        packet_count += 1

    print("Batch orbit estimation completed")

    print("Final state estimate:")
    print(f"\t{orbit_ekf.x}")
    print(f"Final std dev:")
    print(f"\t{np.diag(orbit_ekf.F)}")

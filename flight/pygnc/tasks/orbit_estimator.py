import brahe
import numpy as np
import time


from ..common import data_parsing, transformations, messages
from ..configuration import orbit_estimator as oe_config
from ..configuration import pygnc as pygnc_config
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

def send_orbit_estimate_message(measurement_epoch, orbit_ekf, sensor_message):
    x = orbit_ekf.x
    F_diag = np.diag(orbit_ekf.F) 
    oem = messages.OrbitEstimateMessage(
        epoch=measurement_epoch,
        state_estimate=x[0:6],
        disturbance_estimate=x[6:-1],
        state_variance=F_diag[0:6],
        disturbance_variance=F_diag[6:-1],
        sensor_message = sensor_message,
    )
    print(f"OrbitEstimateMessage:\n {oem.as_tuple}")

def main(port):
    print("Orbit Estimator Task")

    # instantiate orbit estimator
    orbit_ekf = OrbitEKF()

    # batch update orbit estimator
    batch_data = data_parsing.unpack_batch_sensor_gps_file_to_messages_iterable(
        pygnc_config.batch_sensor_gps_filepath
    )
    packet_count = 0
    prev_epoch = None
    for bd in batch_data:
        print(f"Packet count = {packet_count}")
        sensor_messages, gps_message = bd
        prev_epoch = update_orbit_ekf(orbit_ekf, gps_message, prev_epoch)
        send_orbit_estimate_message(prev_epoch, orbit_ekf, sensor_messages[-1])
        packet_count += 1

    print("Batch orbit estimation completed")

    print("Final state estimate:")
    print(f"\t{orbit_ekf.x}")
    print(f"Final std dev:")
    print(f"\t{np.diag(orbit_ekf.F)}")

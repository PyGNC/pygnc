import brahe
import numpy as np
import time


from ..common import data_parsing, transformations, messages
from ..common.zmq_messaging import zmqMessagePublisher
from ..configuration import orbit_estimator as oe_config
from ..configuration import pygnc as pygnc_config
from ..algorithms.RangingEstimator import RangingFilter
def read_ranging_data():
    # batch update orbit estimator
    batch_data = data_parsing.unpack_batch_sensor_ranging_file_to_messages_iterable(
        pygnc_config.batch_sensor_ranging_filepath
    )
    return batch_data

def update_orbit_ranging(ranging_filter, ranging_message, prev_epoch=None):
    # get position and velocity state from gps
    state_measurement_ecef = np.concatenate(
        (ranging_message.position, ranging_message.velocity)
    )
    range_measurement = ranging_message.ranges
    range_ids = ranging_message.range_ids

    measurement_epoch = transformations.gps_week_milliseconds_to_brahe_epoch(
        ranging_message.gps_week, ranging_message.gps_milliseconds
    )
    state_measurement_eci = brahe.frames.sECEFtoECI(
        measurement_epoch, state_measurement_ecef
    )
    if prev_epoch is None:
        # need to initialize ranging state with first measurement
        ranging_filter.initialize_state(state_measurement_eci)
    else:
        dt = measurement_epoch - prev_epoch
        ranging_filter.update(state_measurement_eci, dt)

    return measurement_epoch
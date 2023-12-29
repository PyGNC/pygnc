import brahe
import numpy as np
import time


from ..common import data_parsing, transformations, messages
from ..common.zmq_messaging import zmqMessagePublisher
from ..configuration import orbit_estimator as oe_config
from ..configuration import pygnc as pygnc_config
from ..algorithms.RangingEstimator import BA
from ..configuration import ranging_estimator as ranging_estimator_config

def parse_rng_bin(filename):
    rng_packet = data_parsing.unpack_rng_packet(filename)
    return rng_packet


def read_ranging(ranging_message):
    # get position and velocity state from gps
    state_measurement_ecef = np.concatenate(
        (ranging_message[:,2:8])
    )
    range_measurement = ranging_message[:,-1]
    range_ids = ranging_message[:,-2]

    measurement_epoch = transformations.gps_week_milliseconds_to_brahe_epoch(
        ranging_message[:,0], ranging_message[:,1]
    )
    state_measurements_eci = np.zeros((len(measurement_epoch), 6))
    for k in range(len(measurement_epoch)):
        state_measurements_eci[k,:] = brahe.frames.sECEFtoECI(
            measurement_epoch[k], state_measurement_ecef[k,:]
        )

    return state_measurements_eci, range_measurement, range_ids, measurement_epoch

def send_ranging_estimate_message(
    pub: zmqMessagePublisher, measurement_epoch, ranging_est , sensor_message
):
    x = ranging_est.x
    oem = messages.OrbitEstimateMessage(
        epoch=measurement_epoch,
        state_estimate=x[0:6],
        sensor_message=sensor_message,
    )
    pub.send(oem)

def main():
    # instantiate orbit estimator
    ranging_filter = BA()
    oem_pub = zmqMessagePublisher(messages.RangingEstimateMessage)
    rng_packet = parse_rng_bin(ranging_estimator_config.rng_filepath)
    # batch update orbit estimator
    state_measurements_eci, range_measurement, range_ids, measurement_epoch = read_ranging(rng_packet)
    initial_guess_0 = state_measurements_eci[0,:] + np.array(range_measurement*np.random.randn(3), 10*np.random.randn(3))
    initial_guess = np.zeros((len(measurement_epoch), 6))
    initial_guess[0,:] = initial_guess_0
    for k in range(1,len(measurement_epoch)):
        initial_guess[k,:] = ranging_filter.rk4(initial_guess[k-1,:], measurement_epoch[k]-measurement_epoch[k-1], ranging_filter.process_dynamics)
    x_est = ranging_filter.scipy_solver(state_measurements_eci, initial_guess, range_measurement)
    send_ranging_estimate_message(oem_pub, measurement_epoch, ranging_filter, rng_packet)
    
if __name__ == "__main__":
    main()

    


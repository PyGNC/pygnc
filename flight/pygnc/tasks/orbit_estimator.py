import brahe
import numpy as np
import time

#plotting 
import matplotlib.pyplot as plt

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
        #this was code before (negative)
        #dt = prev_epoch - measurement_epoch
        
        print("in here")

        dt = abs(prev_epoch - measurement_epoch)

        
        orbit_ekf.update(state_measurement_eci, dt)

    return measurement_epoch


def main(batch_gps_sensor_data_filepath):
    print("Orbit Estimator Task")

    # instantiate orbit estimator
    orbit_ekf = OrbitEKF()

    #save all the estimates
    all_estimates = np.zeros((12, 144))

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

        all_estimates[:, packet_count] = orbit_ekf.x

        packet_count += 1

    print("Batch orbit estimation completed")

    print("Final state estimate:")
    print(f"\t{orbit_ekf.x}")
    print(f"Final std dev:")
    print(f"\t{np.diag(orbit_ekf.F)}")

    ground_truth_states = np.loadtxt('/home/fausto/pygnc/scenario_generator/state_history.txt', delimiter=',')

    #size of ground truth states is 36600x13
    #need to down sample to every 25 seconds
    #this is the time array 

    t = np.linspace(0,ground_truth_states.shape[0], num=ground_truth_states.shape[0]) 

    indices = np.arange(0,ground_truth_states.shape[0], step=250)

    print("indices", indices)

    print("size of gt states", ground_truth_states.shape)

    print("size of indices: ", indices.shape)
    #calculate residuals. need to fix some of the indexing to get the dimensions to match
    #residuals = ground_truth_states - all_estimates

    #Plotting the code for testing 
    fig, axs = plt.subplots(3)
    fig.suptitle('State Estimates')
    axs[0].plot(all_estimates[0,:])
    axs[0].plot(ground_truth_states[indices,0])
    axs[0].set_title('x')
    axs[1].plot(all_estimates[1,:])
    axs[1].plot(ground_truth_states[indices,1])
    axs[1].set_title('y')
    axs[2].plot(all_estimates[2,:])
    axs[2].plot(ground_truth_states[indices,2])
    axs[2].set_title('z')

    #save the plot as a png
    fig.savefig('position_estimate.png')
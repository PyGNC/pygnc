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

    #measurement_epoch =  brahe.epoch.Epoch("2020-11-29T23:59:23.0Z")

    print("THIS IS measurement epoch: ", measurement_epoch)

    
    state_measurement_eci = brahe.frames.sECEFtoECI(
        measurement_epoch, state_measurement_ecef
    )

    print("state measurement eci: ", state_measurement_eci)

    if prev_epoch is None:
        # need to initialize ekf state with first measurement
        #print("first measurement: ", state_measurement_eci)

        orbit_ekf.initialize_state(state_measurement_eci)
    else:
        #this was code before (negative)
        #dt = prev_epoch - measurement_epoch
        
        #print("in here")

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

    #20 second delay between ground truth and measurements
    ground_truth_states = np.loadtxt('/home/fausto/pygnc/scenario_generator/state_history_test.txt', delimiter=',')[199:,:]

    #size of ground truth states is 36600x13
    #need to down sample to every 25 seconds
    #this is the time array 

    t = np.linspace(0,ground_truth_states.shape[0], num=ground_truth_states.shape[0]) 

    indices = np.arange(0,ground_truth_states.shape[0], step=250)

    #print("indices", indices)

    print("size of gt states", ground_truth_states.shape)

    print("size of indices: ", indices.shape)

    print("size of estimates", all_estimates.shape)
    #calculate residuals. need to fix some of the indexing to get the dimensions to match

    #residuals = ground_truth_states - all_estimates

    #first residual
    print("first residual", ground_truth_states[indices[0],0:6] - all_estimates[0:6,0])

    plot_size = all_estimates.shape[1]

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


    #RESIDUAL PLOTS
    fig2, axs2 = plt.subplots(3)
    fig2.suptitle('Residuals')
    axs2[0].plot(ground_truth_states[indices[:144],0] - all_estimates[0,:])
    axs2[0].set_title('x')
    axs2[1].plot(ground_truth_states[indices[:144],1] - all_estimates[1,:])
    axs2[1].set_title('y')
    axs2[2].plot(ground_truth_states[indices[:144],2] - all_estimates[2,:])
    axs2[2].set_title('z')


    fig3, axs3 = plt.subplots(3)
    fig3.suptitle('Residuals')
    axs3[0].plot(ground_truth_states[indices[:144],3] - all_estimates[3,:])
    axs3[0].set_title('x')
    axs3[1].plot(ground_truth_states[indices[:144],4] - all_estimates[4,:])
    axs3[1].set_title('y')
    axs3[2].plot(ground_truth_states[indices[:144],5] - all_estimates[5,:])
    axs3[2].set_title('z')


    #save the plot as a png
    fig.savefig('position_estimate.png')
    fig2.savefig('residuals.png')
    fig3.savefig('vel_residuals.png')
import brahe
import numpy as np
import time


from ..common import data_parsing, transformations
from ..configuration import orbit_estimator as oe_config
from ..algorithms.OrbitEstimator import OrbitEKF

def predict_orbit_ekf(orbit_ekf, prev_epoch, dt=5.0):
    orbit_ekf.predict(dt)
    new_epoch = prev_epoch + dt # addition of seconds is defined in brahe
    return new_epoch


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
        dt = measurement_epoch - prev_epoch
        orbit_ekf.predict(dt)
        orbit_ekf.update(state_measurement_eci)

    return measurement_epoch



def main(batch_gps_sensor_data_filepath):
    print("Orbit Estimator Task")

    # instantiate orbit estimator
    orbit_ekf = OrbitEKF()

    # batch update orbit estimator
    batch_data = data_parsing.unpack_batch_sensor_gps_file_to_messages_iterable(
        batch_gps_sensor_data_filepath
    )
    

    # We predict the next state every 1 sec 
    # Every 25 sec, we process the measurement packet with an EKF update
    packet_count = 0
    prev_epoch = None
    in_prediction = 5

    estimates = []
    j = 0
    for bd in batch_data:
        print(f"Packet count = {packet_count}")
        _, gps_message = bd

        prev_epoch = update_orbit_ekf(orbit_ekf, gps_message, prev_epoch)
        packet_count += 1

        estimates.append(orbit_ekf.x)
        for i in range(in_prediction-1):
            prev_epoch = predict_orbit_ekf(orbit_ekf, prev_epoch)
            estimates.append(orbit_ekf.x)

    print("Batch orbit estimation completed")

    """estimates = np.array(estimates)
    plot_size = estimates.shape[0]
    import matplotlib.pyplot as plt
    ground_truth_states = np.loadtxt('/home/ibrahima/CMU/pygnc/scenarios/default_scenario/state_hist.txt', delimiter=',')[199:, :] # 20s delay on the txt file
    #ground_truth_states = np.loadtxt('/home/ibrahima/CMU/pygnc/scenarios/long_scenario/state_hist.txt', delimiter=',')[199:, :]
    #ground_truth_states = ground_truth_states[::250, :6] # every 25 sec
    ground_truth_states = ground_truth_states[::50, :6] # every 5 sec


    fig1, axs1 = plt.subplots(3)
    fig1.suptitle('Position estimation')
    axs1[0].plot(estimates[:,0], color='blue', label='est')
    axs1[0].plot(ground_truth_states[:plot_size,0], color='red', label='gt')
    axs1[0].set_title('x')
    axs1[0].legend() 
    axs1[1].plot(estimates[:,1], color='blue', label='est')
    axs1[1].plot(ground_truth_states[:plot_size,1], color='red', label='gt')
    axs1[1].set_title('y')
    axs1[1].legend() 
    axs1[2].plot(estimates[:,2], color='blue', label='est')
    axs1[2].plot(ground_truth_states[:plot_size,2], color='red', label='gt')
    axs1[2].set_title('z')
    axs1[2].legend() 

    
    fig2, axs2 = plt.subplots(3)
    fig2.suptitle('Velocity estimation')
    axs2[0].plot(estimates[:,3], color='blue', label='est')
    axs2[0].plot(ground_truth_states[:plot_size,3], color='red', label='gt')
    axs2[0].set_title('xdot')
    axs2[0].legend() 
    axs2[1].plot(estimates[:,4], color='blue', label='est')
    axs2[1].plot(ground_truth_states[:plot_size,4], color='red', label='gt')
    axs2[1].set_title('ydot')
    axs2[1].legend() 
    axs2[2].plot(estimates[:,5], color='blue', label='est')
    axs2[2].plot(ground_truth_states[:plot_size,5], color='red', label='gt')
    axs2[2].set_title('zdot')
    axs2[2].legend()

    fig3, axs3 = plt.subplots(2)
    fig3.suptitle('Residual orbit estimation')
    axs3[0].plot(ground_truth_states[:plot_size,0] - estimates[:,0], label='x')
    axs3[0].plot(ground_truth_states[:plot_size,1] - estimates[:,1], label='y')
    axs3[0].plot(ground_truth_states[:plot_size,2] - estimates[:,2], label='z')
    axs3[0].legend() 
    axs3[0].set_title('Position')
    axs3[1].plot(ground_truth_states[:plot_size,3] - estimates[:,3], label='xdot')
    axs3[1].plot(ground_truth_states[:plot_size,4] - estimates[:,4], label='ydot')
    axs3[1].plot(ground_truth_states[:plot_size,5] - estimates[:,5], label='zdot')
    axs3[1].legend() 
    axs3[1].set_title('Velocity')

    folder_path = 'scenarios/default_scenario/'
    #folder_path = 'scenarios/long_scenario/'
    fig1.savefig(folder_path + 'position_estimation.png')
    fig2.savefig(folder_path + 'velocity_estimation.png')
    fig3.savefig(folder_path + 'residuals.png')"""

    print("Final state estimate:")
    print(f"\t{orbit_ekf.x}")
    print(f"Final std dev:")
    print(f"\t{np.diag(orbit_ekf.F)}")

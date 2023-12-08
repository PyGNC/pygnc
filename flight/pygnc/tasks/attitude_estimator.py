#attitude estimator task 
#TODO: FIX the tuning
import brahe
import numpy as np
import time

#plotting 
import matplotlib.pyplot as plt

from ..common import data_parsing, transformations
from ..configuration import orbit_estimator as oe_config
from ..algorithms.AttitudeEstimator import OrbitMEKF

#solar lux + earth albedo lux
total_lux = (1361 * 98) + 0.4 * (1361 * 98)

def get_sun_vector(sun_sensors_normalized): 

    B = np.vstack((np.eye(3), -np.eye(3)))

    #use the pseudoinverse to solve for the sun vector
    sun_vector = np.linalg.solve(B.T@B, B.T@sun_sensors_normalized[:,None])

    #reduce a dimension to sun vector
    sun_vector = sun_vector[:,0]

    #normalize
    sun_vector = sun_vector / np.linalg.norm(sun_vector)

    return sun_vector    

def update_orbit_mekf(orbit_mekf, sensor_message, prev_epoch=None):
    # get position and velocity state from sensor message

    #these are in deg/s
    gyro_measurements = sensor_message.gyro_measurement

    #print("gyro measurements deg/s: ", gyro_measurements)

    #convert to rad/s
    gyro_measurements = gyro_measurements * np.pi/180

    #print("gyro measurements rad/s: ", gyro_measurements)

    mag_measurements = sensor_message.mag_measurement

    #normalize the magnetometer measurement
    mag_measurements = mag_measurements/np.linalg.norm(mag_measurements)

    #unnormalized lux measurements
    sun_measurements = sensor_message.sun_sensors

    sun_measurement_normalized = sun_measurements/ total_lux

    #these sun measurements are normalized
    #print("sun measurements: ", sun_measurements)

    spacecraft_time = sensor_message.spacecraft_time

    #normalized in the function
    sun_vector = get_sun_vector(sun_measurement_normalized)

    #print("this is sun vector: ", sun_vector)

    #normalize the magnetometer measurement
    #mag_measurements = mag_measurements/np.linalg.norm(mag_measurements)

    #print("mag measurements: ", mag_measurements)
    #print("sun vector measurements: ", sun_vector)
    #print("gyro measurements: ", gyro_measurements)

    state_measurement_body = np.hstack([[sun_vector], [mag_measurements]])

    state_measurement_body = state_measurement_body.T[:,0]

    #print("state measurement body: ", state_measurement_body)

    if prev_epoch is None:
        #initialize at an arbitrary attitude
        orbit_mekf.initialize_state()
        orbit_mekf.u = gyro_measurements
    else:
        #dt = prev_epoch - measurement_epoch
        dt = 5; #defined when generating the .bin files
        orbit_mekf.u = gyro_measurements
        orbit_mekf.update(state_measurement_body, dt)

    #return measurement_epoch
    return spacecraft_time

#the filter is initialized with the first measurement and then updated

def main(batch_gps_sensor_data_filepath):
    print("Attitude Estimator Task")

    #save all the estimates
    all_estimates = np.zeros((10, 144*5))

    # instantiate orbit estimator
    orbit_mekf = OrbitMEKF()

    # batch update orbit estimator
    batch_data = data_parsing.unpack_batch_sensor_gps_file_to_messages_iterable(
        batch_gps_sensor_data_filepath
    )

    packet_count = 0
    prev_epoch = None

    count = 0
    for bd in batch_data:
        print(f"Packet count = {packet_count}")

        #this is a list of sensor measurements. The amount of sensor measurments in one update of the GPS is 5
        #Ensure that you pass in the next measurement
        sensor_msg, _ = bd

        for i in range(len(sensor_msg)):
            sensor_msgs = sensor_msg[i]
            prev_epoch = update_orbit_mekf(orbit_mekf, sensor_msgs, prev_epoch)

            #save the state estimate
            all_estimates[:,count] = orbit_mekf.x
            
            count += 1
            print("state estimate:")
            print(f"\t{orbit_mekf.x}")
            print(f"std dev:")
            print(f"\t{np.diag(orbit_mekf.P)}")

        packet_count += 1

    
    print("Batch attitude estimation completed")

    #import the ground truth states txt file
    ground_truth_states = np.loadtxt('/home/fausto/pygnc/scenario_generator/state_history.txt', delimiter=',')

    #print size of ground truth states
    print("ground truth states shape: ", ground_truth_states.shape)

    #The size of attitute estimates is 10x720
    #There are 36600 measurements in the total ground truth states. The dt of the ground truth simulation 
    #is 0.1, so there is about 1 hour of data. 

    print("Final state estimate:")
    print(f"\t{orbit_mekf.x}")
    print(f"Final std dev:")
    print(f"\t{np.diag(orbit_mekf.P)}")

    #Plotting the code for testing 
    #plot the quaternion estimates as subplots
    fig, axs = plt.subplots(4)
    fig.suptitle('Quaternion Estimates')
    axs[0].plot(all_estimates[0,:])
    #axs[0].plot(ground_truth_states[:,6])
    axs[0].set_title('q1')
    axs[1].plot(all_estimates[1,:])
    #axs[1].plot(ground_truth_states[:,7])
    axs[1].set_title('q2')
    axs[2].plot(all_estimates[2,:])
    #axs[2].plot(ground_truth_states[:,8])
    axs[2].set_title('q3')
    axs[3].plot(all_estimates[3,:])
    #axs[3].plot(ground_truth_states[:,9])
    axs[3].set_title('q4')

    #Get the ground truth for the gyro bias of that scenerio
    #plot the gyro bias estimates as subplots
    fig2, axs = plt.subplots(3)
    fig2.suptitle('Gyro Bias Estimates')
    axs[0].plot(all_estimates[4,:])
    axs[0].set_title('x bias')
    axs[1].plot(all_estimates[5,:])
    axs[1].set_title('y bias')
    axs[2].plot(all_estimates[6,:])
    axs[2].set_title('z bias')

    #Get the ground truth for the gyro bias of that scenerio
    #plot the magnetometer bias estimates as subplots
    fig3, axs = plt.subplots(3)
    fig3.suptitle('Magnetometer Bias Estimates')
    axs[0].plot(all_estimates[7,:])
    axs[0].set_title('x bias')
    axs[1].plot(all_estimates[8,:])
    axs[1].set_title('y bias')
    axs[2].plot(all_estimates[9,:])
    axs[2].set_title('z bias')


    #show the plot
    plt.show()

    #save the plot as a png
    fig.savefig('state_estimates.png')
    fig2.savefig('gyro_bias_estimates.png')
    fig3.savefig('magnetometer_bias_estimates.png')
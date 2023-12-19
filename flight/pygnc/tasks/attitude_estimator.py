#attitude estimator task 
#TODO: FIX the tuning
import brahe
import numpy as np
import time
import math

#plotting 
import matplotlib.pyplot as plt

from ..common import data_parsing, transformations
from ..configuration import orbit_estimator as oe_config
from ..algorithms.AttitudeEstimator import OrbitMEKF
from ..algorithms.AttitudeEstimator import inertial_models
from ..algorithms.AttitudeEstimator import OrbitSQMEKF

from ..algorithms.AttitudeEstimator import OrbitSQMEKF_nb

from ..algorithms.AttitudeEstimator.mekf_utils import *

#solar lux + earth albedo lux
total_lux = (1361 * 98) + 0.4 * (1361 * 98)

#for the new default scenerio
#mag_bias_truth = np.array([15.16648, 12.85475, -36.121958])

#for the longer horizon
mag_bias_truth = np.array([-23.10390, 8.793605, 36.207927])

# def get_sun_vector(sun_sensors_normalized): 

#     B = np.vstack((np.eye(3), -np.eye(3)))

#     #use the pseudoinverse to solve for the sun vector
#     sun_vector = np.linalg.solve(B.T@B, B.T@sun_sensors_normalized[:,None])

#     #reduce a dimension to sun vector
#     sun_vector = sun_vector[:,0]

#     #normalize
#     sun_vector = sun_vector / np.linalg.norm(sun_vector)

#     return sun_vector  

#import the ground truth states txt file
#for the new default scenerio
#ground_truth_states = np.loadtxt('/home/fausto/pygnc/scenario_generator/state_history.txt', delimiter=',')

ground_truth_states = np.loadtxt('/home/fausto/pygnc/scenario_generator/state_history_long.txt', delimiter=',')

groundtruth_attitude = np.loadtxt('/home/fausto/pygnc/scenario_generator/sim_attitude_omega_v2_new_orbit.txt', delimiter=',')

#print('ground truth states: ', ground_truth_states.shape)

#this is from the imu frame to body
R_body_IMU = np.array([[0.0, 1.0, 0.0],
                    [-1.0, 0.0, 0.0],
                    [0.0, 0.0, -1.0]])


def get_sun_vector(sun_sensors):

    sun_positive = sun_sensors[0:3]
    sun_negative = sun_sensors[3:]

    #positive face lux values - negative face lux values
    sun_vector_body = sun_positive - sun_negative

    #normalize 
    sun_vector_body = sun_vector_body/np.linalg.norm(sun_vector_body)

    return sun_vector_body


def update_orbit_mekf(orbit_mekf, sensor_message, gps_message,count, prev_epoch=None, prev_spacecraft_time=None):
    # get the epoch of the measurement
    measurement_epoch = transformations.gps_week_milliseconds_to_brahe_epoch(
        gps_message.gps_week, gps_message.gps_milliseconds
    )

    #these are in deg/s
    gyro_measurements = sensor_message.gyro_measurement

    #print("gyro measurements deg/s: ", gyro_measurements)

    #convert to rad/s and into body coordinates
    gyro_measurements = (R_body_IMU@gyro_measurements) * np.pi/180

    #print("gyro measurements rad/s: ", gyro_measurements)

    #get the magnetoemter measurements in the body frame

    #subtract out the magnetometer bias truth value
    mag_measurements = R_body_IMU@sensor_message.mag_measurement #- mag_bias_truth

    #normalize the magnetometer measurement. in the body frame. true body measurement
    mag_vector = mag_measurements/np.linalg.norm(mag_measurements)

    #unnormalized lux measurements
    sun_measurements = sensor_message.sun_sensors

    #sun_measurement_normalized = sun_measurements/ total_lux

    #normalized in the function
    sun_vector = get_sun_vector(sun_measurements)

    #lux measurements stacked with the magnetometer measurements
    all_raw_measurements = np.hstack((sun_measurements, mag_measurements))

    #print("this is sun vector: ", sun_vector)

    #normalize the magnetometer measurement
    #mag_measurements = mag_measurements/np.linalg.norm(mag_measurements)

    #print("mag measurements: ", mag_measurements)
    #print("sun vector measurements: ", sun_vector)
    #print("gyro measurements: ", gyro_measurements)

    #inertial vectors #this would be from the kalman predictions as well as updates
     
    #this is the true measurements (body vectors)
    state_measurement_body = np.hstack([[sun_vector], [mag_vector]])

    state_measurement_body = state_measurement_body.T[:,0]

    print("state measurement body: ", state_measurement_body)

    if prev_epoch is None and prev_spacecraft_time is None:
        #initialize at an arbitrary attitude
        orbit_mekf.initialize_state()

        #need to convert the gyro measurements to rad/s
        orbit_mekf.u = (R_body_IMU@gyro_measurements)*(np.pi/180)

        current_epoch = measurement_epoch
        current_spacecraft_time = sensor_message.spacecraft_time

    else:
        current_spacecraft_time = sensor_message.spacecraft_time
        dt = abs(current_spacecraft_time - prev_spacecraft_time)

        #print("this is dt: ", dt)
        #avoid hard coding dt because dt can be variable...
        current_epoch = prev_epoch + dt
        
        #print("this is current epoch: ", current_epoch)
        sun_vector_inertial = brahe.sun_position(current_epoch)
        
        #50 because that is every 5 seconds in the ground truth states array. This is because that was sampled with a dt of 0.1 s
        mag_vector_inertial = inertial_models.IGRF13(ground_truth_states[count*50,0:3], current_epoch)*1e6 #converted to uT 

        #print("mag vector inertial: ", mag_vector_inertial)
        #pass in not normalized. The measurement function takes care of it
        inertial_measurement = np.hstack((sun_vector_inertial, mag_vector_inertial))

        #print("inertial measurement: ", inertial_measurement)
        #need to convert the gyro measurements to rad/s and into body coordinates
        orbit_mekf.u = (R_body_IMU@gyro_measurements)*(np.pi/180)

        orbit_mekf.update(all_raw_measurements, state_measurement_body, inertial_measurement, dt)

    return current_epoch, current_spacecraft_time

    #return measurement_epoch
    #return spacecraft_time

#the filter is initialized with the first measurement and then updated

def main(batch_gps_sensor_data_filepath):
    print("Attitude Estimator Task")

    #save all the estimates
    #for no bias
    #all_estimates = np.zeros((7, 144*5*3))

    #with mag bias
    all_estimates = np.zeros((10, 144*5*3))

    # instantiate orbit estimator
    #orbit_mekf = OrbitMEKF()

    #square root version
    orbit_mekf = OrbitSQMEKF()

    #no magnetomter bias version
    #orbit_mekf = OrbitSQMEKF_nb()

    # batch update orbit estimator
    batch_data = data_parsing.unpack_batch_sensor_gps_file_to_messages_iterable(
        batch_gps_sensor_data_filepath
    )

    packet_count = 0
    prev_epoch = None
    prev_spacecraft_time = None
    count = 0

    for bd in batch_data:
        print(f"Packet count = {packet_count}")

        #this is a list of sensor measurements. The amount of sensor measurments in one update of the GPS is 5
        #Ensure that you pass in the next measurement

        sensor_msg, gps_message = bd

        for i in range(len(sensor_msg)):
            sensor_msgs = sensor_msg[i]
            prev_epoch, prev_spacecraft_time = update_orbit_mekf(orbit_mekf, sensor_msgs, gps_message, count, prev_epoch, prev_spacecraft_time)

            #save the state estimate
            all_estimates[:,count] = orbit_mekf.x
            
            count += 1
            print("state estimate:")
            print(f"\t{orbit_mekf.x}")
            print(f"std dev:")
            #regular version
            #print(f"\t{np.diag(orbit_mekf.P)}")
            #square root version
            print(f"\t{np.diag(orbit_mekf.F)}")

        packet_count += 1

    print("Batch attitude estimation completed")

    #print size of all estimates
    print("all estimates shape: ", all_estimates.shape)
    #print size of ground truth states
    print("ground truth states shape: ", ground_truth_states.shape)

    #The size of attitute estimates is 10x720
    #There are 36600 measurements in the total ground truth states. The dt of the ground truth simulation 
    #is 0.1, so there is about 1 hour of data. 

    #t = np.linspace(0,ground_truth_states.shape[0], num=ground_truth_states.shape[0]) 

    indices = np.arange(0,ground_truth_states.shape[0], step=50)

    print("this is indices 0: ", indices[0])

    print("this is indices 1: ", indices[1])

    attitude_error = np.zeros(all_estimates.shape[1])
    quat_diff = np.zeros((4, all_estimates.shape[1]))

    print("Final state estimate:")
    print(f"\t{orbit_mekf.x}")
    print(f"Final std dev:")
    #regular version
    #print(f"\t{np.diag(orbit_mekf.P)}")
    #square root version
    print(f"\t{np.diag(orbit_mekf.F)}")


    #plotting the attitude error 
    for i in range(all_estimates.shape[1]):

    #inv_truth = conj_q(satellite_attitude_omega[0:4, i])
    #quat_diff[:,i] = L(inv_truth)@all_states[0:4, i]

        inv_all_states = conj_q(all_estimates[0:4, i])
        #quat_diff = L(np.flipud(ground_truth_states[indices[i], 6:10]))@inv_all_states
        quat_diff = L(ground_truth_states[indices[i], 6:10])@inv_all_states
        axisangle_diff = 2*math.acos(quat_diff[0])*180/np.pi
        attitude_error[i] = axisangle_diff
        #in degrees
        #attitude_error[i] = np.linalg.norm(mrp_diff)#*180/np.pi


    #this is is for the new default scenerio
    #ground truth mag bias and gyro bias
    #mag_bias_truth = np.array([15.16648, 12.85475, -36.121958])
    #this is in degrees/second
    #gyro_bias_truth = np.array([1.59531306, -0.61455357, 0.30912115])

    #this is for the long scenerio
    mag_bias_truth = np.array([-23.10390, 8.793605, 36.207927])
    #this is in degrees/second
    gyro_bias_truth = np.array([-2.743841, 0.8487285, -1.412131])

    print("SIZE OF ground truth attitude:", groundtruth_attitude.shape)
    #convert to rad/s
    gyro_bias_truth = gyro_bias_truth*np.pi/180

    print("ground truth sampled size: ", ground_truth_states[indices,6:].shape)

    print("this is first ground truth: ", ground_truth_states[0, 6:10])

    t_size =ground_truth_states[indices,6:].shape[0]


    print("THIS IS THE FIRST GT STATE: ", ground_truth_states[0:6:10])
    #Plotting the code for testing 
    #plot the quaternion estimates as subplots
    fig1, axs1 = plt.subplots(4)
    fig1.suptitle('Quaternion Estimates')
    axs1[0].plot(all_estimates[0,:])
    axs1[0].plot(ground_truth_states[indices,6])
    #axs1[0].plot(ground_truth_states[indices,9])
    axs1[0].set_title('q1')
    axs1[1].plot(all_estimates[1,:])
    axs1[1].plot(ground_truth_states[indices,7])
    #axs1[1].plot(ground_truth_states[indices,8])
    axs1[1].set_title('q2')
    axs1[2].plot(all_estimates[2,:])
    axs1[2].plot(ground_truth_states[indices,8])
    #axs1[2].plot(ground_truth_states[indices,7])
    axs1[2].set_title('q3')
    axs1[3].plot(all_estimates[3,:])
    axs1[3].plot(ground_truth_states[indices,9])
    #axs1[3].plot(ground_truth_states[indices,6])
    axs1[3].set_title('q4')

    fig7, axs7 = plt.subplots(4)
    fig7.suptitle = ('Ground Truth Quaternion')
    axs7[0].plot(ground_truth_states[indices,6])
    axs7[1].plot(ground_truth_states[indices,7])
    axs7[2].plot(ground_truth_states[indices,8])
    axs7[3].plot(ground_truth_states[indices,9])


    #Get the ground truth for the gyro bias of that scenerio
    #plot the gyro bias estimates as subplots
    fig2, axs2 = plt.subplots(3)
    fig2.suptitle('Gyro Bias Estimates')
    axs2[0].plot(all_estimates[4,:])
    axs2[0].plot(gyro_bias_truth[0]*np.ones(t_size))
    axs2[0].set_title('x bias')
    axs2[1].plot(all_estimates[5,:])
    axs2[1].plot(gyro_bias_truth[1]*np.ones(t_size))
    axs2[1].set_title('y bias')
    axs2[2].plot(all_estimates[6,:])
    axs2[2].plot(gyro_bias_truth[2]*np.ones(t_size))
    axs2[2].set_title('z bias')

    #Get the ground truth for the gyro bias of that scenerio
    #plot the magnetometer bias estimates as subplots
    # fig3, axs3 = plt.subplots(3)
    # fig3.suptitle('Magnetometer Bias Estimates')
    # axs3[0].plot(all_estimates[7,:])
    # axs3[0].plot(mag_bias_truth[0]*np.ones(t_size))
    # axs3[0].set_title('x bias')
    # axs3[1].plot(all_estimates[8,:])
    # axs3[1].plot(mag_bias_truth[1]*np.ones(t_size))
    # axs3[1].set_title('y bias')
    # axs3[2].plot(all_estimates[9,:])
    # axs3[2].plot(mag_bias_truth[2]*np.ones(t_size))
    # axs3[2].set_title('z bias')

    fig4, axs4 = plt.subplots(1)
    fig4.suptitle('Attitude Error')
    axs4.plot(attitude_error)


    #show the plot
    #plt.show()

    #save the plot as a png
    fig1.savefig('state_estimates.png')
    fig2.savefig('gyro_bias_estimates.png')
    #fig3.savefig('magnetometer_bias_estimates.png')
    fig4.savefig('attitude_error.png')
    fig7.savefig('groundtruth_quaternion.png')
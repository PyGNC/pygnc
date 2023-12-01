#attitude estimator task 

#TODO: Fill in this file with the appropriate measurements from the .bin files
#TODO: FIX the tuning
import brahe
import numpy as np
import time

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

    spacecraft_time = sensor_message.spacecraft_time

    #normalized in the function
    sun_vector = get_sun_vector(sun_measurement_normalized)

    #normalize the magnetometer measurement
    mag_measurements = mag_measurements/np.linalg.norm(mag_measurements)

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


def main(batch_gps_sensor_data_filepath):
    print("Attitude Estimator Task")

    # instantiate orbit estimator
    orbit_mekf = OrbitMEKF()

    # batch update orbit estimator
    batch_data = data_parsing.unpack_batch_sensor_gps_file_to_messages_iterable(
        batch_gps_sensor_data_filepath
    )
    packet_count = 0
    prev_epoch = None
    for bd in batch_data:
        print(f"Packet count = {packet_count}")

        #this is a list of sensor measurements. The amount of sensor measurments in one update of the GPS
        sensor_msg, _ = bd

        for i in range(len(sensor_msg)):
            sensor_msgs = sensor_msg[i]
            prev_epoch = update_orbit_mekf(orbit_mekf, sensor_msgs, prev_epoch)

        packet_count += 1

    print("Batch attitude estimation completed")

    print("Final state estimate:")
    print(f"\t{orbit_mekf.x}")
    print(f"Final std dev:")
    print(f"\t{np.diag(orbit_mekf.P)}")

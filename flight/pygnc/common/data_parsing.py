import struct
import numpy as np
from ..common import constants


def unpack_batch_gps_message(gps_packet):
    """unpack_batch_gps_message
    Unpack a single gps message that is formatted as in a batch data file.
    """

    spacecraft_time = int.from_bytes(gps_packet[0:4], "big")
    gps_frame = gps_packet[4:]
    ecef = struct.unpack("<dddfffBdddffffffBB", gps_frame[8:])
    gps_data = (
        spacecraft_time,  # approx spacecraft time at gps frame (within 0.8s or less)
        gps_frame[0],  # GPS time sol status
        int.from_bytes(gps_frame[1:3], "little"),  # GPS week
        int.from_bytes(gps_frame[3:7], "little"),  # time of week (milliseconds)
        gps_frame[7],  # pos sol status
        *ecef,
    )

    return gps_data


def opt3001_raw_to_lux(raw):
    exponent = (raw & 0xF000) >> 12
    fractional = raw & 0x0FFF
    lux = (0.01 * fractional) * (2**exponent)
    return lux


def unpack_batch_sensor_message(sensor_packet):
    """
    unpack a single sensor message that is formatted as in a batch data file.
    """
    spacecraft_time = int.from_bytes(sensor_packet[0:4], "big")
    imu_data = struct.unpack("<hhhhhhh", sensor_packet[4:18])
    mag_measurement = np.array(imu_data[0:3]) * constants.mag_scalar_raw_to_uT
    raw_hall = imu_data[3]
    gyro_measurement = np.array(imu_data[4:7]) * constants.gyro_scalar_raw_to_deg_s
    sun_raw = struct.unpack(">HHHHHH", sensor_packet[18:30])
    sun_lux = [opt3001_raw_to_lux(sun_raw_i) for sun_raw_i in sun_raw]

    return (spacecraft_time, mag_measurement, raw_hall, gyro_measurement, sun_lux)


def unpack_batch_sensor_gps_message(sensor_gps_packet):
    # last two bytes should be \r\n
    if not sensor_gps_packet[-2:] == b"\r\n":
        raise ValueError(
            "Sensor packet improperly formed, terminating `\\r\\n` missing"
        )
    sensor_gps_packet = sensor_gps_packet[0:-2]  # remove terminating characters
    if not len(sensor_gps_packet) == constants.batch_packet_expected_packet_length:
        raise ValueError("Sensor packet improperly formed, improper length")

    len_sensor = constants.batch_sensor_packet_length_bytes
    N_sensor = constants.batch_packet_num_sensor_packets
    sensor_data_list = []
    for i in range(N_sensor):
        sensor_packet_i = sensor_gps_packet[i * len_sensor : (i + 1) * len_sensor]
        sensor_data_list.append(unpack_batch_sensor_message(sensor_packet_i))
    gps_packet = sensor_gps_packet[N_sensor * len_sensor :]
    gps_data = unpack_batch_gps_message(gps_packet)

    return sensor_data_list, gps_data

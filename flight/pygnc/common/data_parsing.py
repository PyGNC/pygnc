import struct
import numpy as np

from . import constants
from . import messages


def unpack_batch_gps_packet(gps_packet):
    """unpack_batch_gps_packet
    Unpack a single gps packet that is formatted as in a batch data file.
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


def unpack_batch_gps_packet_to_message(gps_packet):
    """
    Convert from raw gps packet to GPSMessage
    """
    return batch_gps_data_to_message(unpack_batch_gps_packet(gps_packet))


def batch_gps_data_to_message(gps_data):
    """
    Convert the output of `unpack_batch_gps_packet` to GPSMessage
    """
    return messages.GPSMessage(
        spacecraft_time=gps_data[0],
        gps_time_status=gps_data[1],
        gps_week=gps_data[2],
        gps_milliseconds=gps_data[3],
        position_status=gps_data[4],
        position_x=gps_data[5],
        position_y=gps_data[6],
        position_z=gps_data[7],
        position_sig_x=gps_data[8],
        position_sig_y=gps_data[9],
        position_sig_z=gps_data[10],
        velocity_status=gps_data[11],
        velocity_x=gps_data[12],
        velocity_y=gps_data[13],
        velocity_z=gps_data[14],
        velocity_sig_x=gps_data[15],
        velocity_sig_y=gps_data[16],
        velocity_sig_z=gps_data[17],
        v_latency=gps_data[18],
        differential_age=gps_data[19],
        solution_age=gps_data[20],
        sats_tracked=gps_data[21],
        sats_in_solution=gps_data[22],
    )


def opt3001_raw_to_lux(raw):
    exponent = (raw & 0xF000) >> 12
    fractional = raw & 0x0FFF
    lux = (0.01 * fractional) * (2**exponent)
    return lux


def unpack_batch_sensor_packet(sensor_packet):
    """
    unpack a single sensor packet that is formatted as in a batch data file.
    """
    spacecraft_time = int.from_bytes(sensor_packet[0:4], "big")
    imu_data = struct.unpack("<hhhhhhh", sensor_packet[4:18])
    mag_measurement = np.array(imu_data[0:3]) * constants.mag_scalar_raw_to_uT
    raw_hall = imu_data[3]
    gyro_measurement = np.array(imu_data[4:7]) * constants.gyro_scalar_raw_to_deg_s
    sun_raw = struct.unpack(">HHHHHH", sensor_packet[18:30])
    sun_lux = [opt3001_raw_to_lux(sun_raw_i) for sun_raw_i in sun_raw]

    return (spacecraft_time, mag_measurement, raw_hall, gyro_measurement, sun_lux)


def unpack_batch_sensor_packet_to_message(sensor_packet):
    """
    unpack a single sensor packet and return a SensorMessage
    """
    return batch_sensor_data_to_message(unpack_batch_sensor_packet(sensor_packet))


def batch_sensor_data_to_message(sensor_data):
    """
    Take the tuple output by `unpack_batch_sensor_data_packet`
    and convert it to a SensorMessage
    """
    return messages.SensorMessage(
        spacecraft_time=sensor_data[0],
        mag_measurement=sensor_data[1],
        raw_hall=sensor_data[2],
        gyro_measurement=sensor_data[3],
        sun_sensors=sensor_data[4],
    )


def unpack_batch_sensor_gps_packet(sensor_gps_packet):
    len_full = constants.batch_sensor_gps_packet_expected_packet_length
    if not len(sensor_gps_packet) == len_full:
        raise ValueError("Sensor packet improperly formed, improper length")

    # last two bytes should be \r\n
    terminating_bytes = constants.batch_sensor_gps_packet_terminating_bytes
    if not sensor_gps_packet[-2:] == terminating_bytes:
        raise ValueError("Sensor packet improperly formed, terminating bytes missing")

    sensor_gps_packet = sensor_gps_packet[0:-2]  # remove terminating characters

    len_sensor = constants.batch_sensor_packet_length_bytes
    N_sensor = constants.batch_sensor_gps_packet_num_sensor_packets
    sensor_data_list = []
    for i in range(N_sensor):
        sensor_packet_i = sensor_gps_packet[i * len_sensor : (i + 1) * len_sensor]
        sensor_data_list.append(unpack_batch_sensor_packet(sensor_packet_i))
    gps_packet = sensor_gps_packet[N_sensor * len_sensor :]
    gps_data = unpack_batch_gps_packet(gps_packet)

    return sensor_data_list, gps_data


def unpack_batch_sensor_gps_packet_to_messages(sensor_gps_packet):
    sensor_data_list, gps_data = unpack_batch_sensor_gps_packet(sensor_gps_packet)
    sensor_message_list = []
    for sensor_data in sensor_data_list:
        sensor_message_list.append(
            messages.SensorMessage(
                spacecraft_time=sensor_data[0],
                mag_measurement=sensor_data[1],
                raw_hall=sensor_data[2],
                gyro_measurement=sensor_data[3],
                sun_sensors=sensor_data[4],
            )
        )
    gps_message = batch_gps_data_to_message(gps_data)

    return sensor_message_list, gps_message


def unpack_batch_sensor_gps_file_to_messages_iterable(batch_file_path):
    """
    Read and return one "line" of sensor and gps messages from the batch sensor gps file
    Returns an iterable that can be iterated in a for loop
    """
    packet_length = constants.batch_sensor_gps_packet_expected_packet_length
    buffer = bytearray(packet_length)
    packet_vbuff = memoryview(buffer)
    with open(batch_file_path, "rb") as file:
        while True:
            bytes_read = file.readinto(packet_vbuff)
            if not bytes_read == packet_length:
                break
            (
                sensor_message_list,
                gps_message,
            ) = unpack_batch_sensor_gps_packet_to_messages(packet_vbuff)
            yield sensor_message_list, gps_message

# One place for constants that are used throughout the code

mag_scalar_raw_to_uT = 1 / 16.0
gyro_scalar_raw_to_deg_s = 1 / 65.6

batch_sensor_packet_length_bytes = 30
batch_gps_packet_length_bytes = 99
batch_sensor_gps_packet_num_sensor_packets = 5
batch_sensor_gps_packet_terminating_bytes = bytearray(b"\r\n")
batch_sensor_gps_packet_terminating_bytes_length = len(
    batch_sensor_gps_packet_terminating_bytes
)
batch_sensor_gps_packet_expected_packet_length = (
    batch_sensor_gps_packet_num_sensor_packets * batch_sensor_packet_length_bytes
    + batch_gps_packet_length_bytes
    + batch_sensor_gps_packet_terminating_bytes_length
)
batch_sensor_gps_file_name = "batch_sensor_gps_data.bin"


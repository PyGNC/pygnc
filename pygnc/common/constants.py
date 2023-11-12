# One place for constants that are used throughout the code

mag_scalar_raw_to_uT = 1 / 16.0
gyro_scalar_raw_to_deg_s = 1 / 65.6

batch_sensor_packet_length_bytes = 30
batch_gps_packet_length_bytes = 99
batch_packet_num_sensor_packets = 5
batch_packet_expected_packet_length = (
    batch_packet_num_sensor_packets * batch_sensor_packet_length_bytes
    + batch_gps_packet_length_bytes
)

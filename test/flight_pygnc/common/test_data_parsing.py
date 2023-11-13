import os
import unittest

from .. import context
from pygnc.common import data_parsing  # type: ignore
from pygnc.common import constants  # type: ignore


class TestDataParsing(unittest.TestCase):
    def setUp(self):
        # test_pkt generated from data_packing.jl
        self.gps_test_pkt = bytearray(
            b"eK\x954\xa0\xef\x08a3w\x12\x01\x00\x00\x00\x00\x80\x84.A\x00\x00\x00\x00\x80\x84>A\x00\x00\x00\x00`\xe3FA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x00\x00\x00\x90\x9a@\x00\x00\x00\x00\x00 \x9c@\x00\x00\x00\x00\x00\xb0\x9d@\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\n\x08"
        )

        self.sensor_test_pkt = bytearray(
            b"eK\x954\x9d\x00\x16\x043\x00\x00\x00\xe0\x02\xda\x00\x1d\x00\x04\xbfM\xac\xac5\\\x03\x00\x00\xbf\xff"
        )

    def test_unpack_gps(self):
        gps_data = data_parsing.unpack_batch_gps_packet(self.gps_test_pkt)
        self._verify_gps_test_packet(gps_data)

    def _verify_gps_test_packet(self, gps_data):
        (
            spacecraft_time,
            gps_time_status,
            gps_week,
            gps_milliseconds,
            position_status,
            position_x,
            position_y,
            position_z,
            position_sig_x,
            position_sig_y,
            position_sig_z,
            velocity_status,
            velocity_x,
            velocity_y,
            velocity_z,
            velocity_sig_x,
            velocity_sig_y,
            velocity_sig_z,
            v_latency,
            differential_age,
            solution_age,
            sats_tracked,
            sats_in_solution,
        ) = gps_data

        gps_millis_in, gps_week_in = (309801825, 2287)
        unix_time_in = 1.699452212825686e9
        position_in = [1e6, 2e6, 3e6]
        velocity_in = [17e2, 18e2, 19e2]

        self.assertEqual(spacecraft_time, int(unix_time_in))
        self.assertEqual(gps_time_status, 160)
        self.assertEqual(gps_week, gps_week_in)
        self.assertEqual(gps_milliseconds, gps_millis_in)
        self.assertEqual(position_status, 1)
        self.assertAlmostEqual(position_in[0], position_x)
        self.assertAlmostEqual(position_in[1], position_y)
        self.assertAlmostEqual(position_in[2], position_z)
        self.assertAlmostEqual(0.0, position_sig_x)
        self.assertAlmostEqual(0.0, position_sig_y)
        self.assertAlmostEqual(0.0, position_sig_z)
        self.assertEqual(velocity_status, 1)
        self.assertAlmostEqual(velocity_in[0], velocity_x)
        self.assertAlmostEqual(velocity_in[1], velocity_y)
        self.assertAlmostEqual(velocity_in[2], velocity_z)
        self.assertAlmostEqual(0.0, velocity_sig_x)
        self.assertAlmostEqual(0.0, velocity_sig_y)
        self.assertAlmostEqual(0.0, velocity_sig_z)
        self.assertEqual(v_latency, 0.0)
        self.assertEqual(differential_age, 0.0)
        self.assertEqual(solution_age, 0.0)
        self.assertEqual(sats_tracked, 10)
        self.assertEqual(sats_in_solution, 8)

    def test_unpack_sensors(self):
        sensor_data = data_parsing.unpack_batch_sensor_packet(self.sensor_test_pkt)
        self._verify_sensor_test_packet(sensor_data)

    def _verify_sensor_test_packet(self, sensor_data):
        unix_timestamp_in = 1.699452212825686e9
        mag_measurement_in = [9.87, 65.43, 3.21]
        gyro_measurement_in = [11.22, 3.33, 0.4433]
        sun_sensors_in_sensor_binned = [12.15, 560.0, 32000.0, 984.0, 0.0, 83865.6]

        (
            spacecraft_time,
            mag_measurement_out,
            raw_hall_out,
            gyro_measurement_out,
            sun_sensors_out,
        ) = sensor_data

        # fmt: off
        self.assertEqual(spacecraft_time, int(unix_timestamp_in))
        self.assertAlmostEqual(mag_measurement_in[0], mag_measurement_out[0], places=0)
        self.assertAlmostEqual(mag_measurement_in[1], mag_measurement_out[1], places=0)
        self.assertAlmostEqual(mag_measurement_in[2], mag_measurement_out[2], places=0)
        self.assertEqual(raw_hall_out, 0)
        self.assertAlmostEqual(gyro_measurement_in[0], gyro_measurement_out[0], places=1)
        self.assertAlmostEqual(gyro_measurement_in[1], gyro_measurement_out[1], places=1)
        self.assertAlmostEqual(gyro_measurement_in[2], gyro_measurement_out[2], places=1)
        self.assertAlmostEqual(sun_sensors_in_sensor_binned[0], sun_sensors_out[0])
        self.assertAlmostEqual(sun_sensors_in_sensor_binned[1], sun_sensors_out[1])
        self.assertAlmostEqual(sun_sensors_in_sensor_binned[2], sun_sensors_out[2])
        self.assertAlmostEqual(sun_sensors_in_sensor_binned[3], sun_sensors_out[3])
        self.assertAlmostEqual(sun_sensors_in_sensor_binned[4], sun_sensors_out[4])
        self.assertAlmostEqual(sun_sensors_in_sensor_binned[5], sun_sensors_out[5])
        # fmt: on

    def test_unpack_sensor_gps(self):
        sensor_gps_test_pkt = bytearray(b"")
        for i in range(constants.batch_sensor_gps_packet_num_sensor_packets):
            sensor_gps_test_pkt += self.sensor_test_pkt
        sensor_gps_test_pkt += self.gps_test_pkt
        sensor_gps_test_pkt += bytearray(b"\r\n")

        sensor_data_list, gps_data = data_parsing.unpack_batch_sensor_gps_packet(
            sensor_gps_test_pkt
        )

        self.assertEqual(
            len(sensor_data_list), constants.batch_sensor_gps_packet_num_sensor_packets
        )

        for sensor_data_i in sensor_data_list:
            self._verify_sensor_test_packet(sensor_data_i)
        self._verify_gps_test_packet(gps_data)

    def test_batch_file_read(self):
        batch_file_path = os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "..",
            "scenarios",
            "unittest_scenario",
            constants.batch_gps_sensor_file_name,
        )

        batch_data = data_parsing.unpack_batch_sensor_gps_file_to_messages_iterable(
            batch_file_path
        )
        packet_count = 0
        for bd_line in batch_data:
            sensor_message_list, gps_message = bd_line
            self.assertEqual(len(sensor_message_list), 5)
            packet_count += 1

        self.assertEqual(packet_count, 5)  # one at 0, 25, 50, 75, 100 seconds

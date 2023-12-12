import numpy as np
import unittest

from .. import context
from pygnc.common import messages  # type: ignore


class TestSensorMessage(unittest.TestCase):
    def test_empty(self):
        sm = messages.SensorMessage()
        self._verify_empty_sensor_message(sm)

    def _verify_empty_sensor_message(self, sm):
        assertNaNs = lambda x: self.assertTrue(np.all(np.isnan(x)))
        assertNP = lambda x: self.assertTrue(isinstance(x, np.ndarray))

        assertNaNs(sm.spacecraft_time)
        assertNaNs(sm.mag_measurement)
        assertNaNs(sm.raw_hall)
        assertNaNs(sm.gyro_measurement)
        assertNaNs(sm.sun_sensors)

        assertNP(sm.mag_measurement)
        assertNP(sm.gyro_measurement)
        assertNP(sm.sun_sensors)

        self.assertEqual(len(sm.mag_measurement), 3)
        self.assertEqual(len(sm.gyro_measurement), 3)
        self.assertEqual(len(sm.sun_sensors), 6)

        self.assertEqual(len(sm.as_tuple), 5)

    def test_empty_msgpack(self):
        sm = messages.SensorMessage()
        b = sm.to_msgpack_b()
        self.assertIsInstance(b, bytes)

        sm2 = messages.SensorMessage(msgpack_b=b)

        self._verify_empty_sensor_message(sm2)

    def test_valid_data(self):
        sc_time = 65432123
        mag = [-1.23, 3.45, 7.7e7]
        raw_hall = 0.0
        gyro = [0.321, -6.54, 34]
        sun = [0.0, 111, 1e4, 3.45e2, 0.01, 84e3]

        sm = messages.SensorMessage(
            sc_time,
            mag,
            raw_hall,
            gyro,
            sun,
        )

        np.testing.assert_array_almost_equal(mag, sm.mag_measurement)
        np.testing.assert_array_almost_equal(gyro, sm.gyro_measurement)
        np.testing.assert_array_almost_equal(sun, sm.sun_sensors)

        np.testing.assert_almost_equal(sc_time, sm.spacecraft_time)
        np.testing.assert_almost_equal(raw_hall, sm.raw_hall)

    def test_valid_data_msgpack(self):
        sc_time = 65432123
        mag = [-1.23, 3.45, 7.7e7]
        raw_hall = 0.0
        gyro = [0.321, -6.54, 34]
        sun = [0.0, 111, 1e4, 3.45e2, 0.01, 84e3]

        sm = messages.SensorMessage(
            sc_time,
            mag,
            raw_hall,
            gyro,
            sun,
        )

        b = sm.to_msgpack_b()
        sm2 = messages.SensorMessage(msgpack_b=b)

        np.testing.assert_array_almost_equal(mag, sm2.mag_measurement)
        np.testing.assert_array_almost_equal(gyro, sm2.gyro_measurement)
        np.testing.assert_array_almost_equal(sun, sm2.sun_sensors)

        np.testing.assert_almost_equal(sc_time, sm2.spacecraft_time)
        np.testing.assert_almost_equal(raw_hall, sm2.raw_hall)


class TestGPSMessage(unittest.TestCase):
    def test_empty(self):
        gm = messages.GPSMessage()
        self._verify_empty_message(gm)

    def _verify_empty_message(self, gm):
        assertNaNs = lambda x: self.assertTrue(np.all(np.isnan(x)))
        assertNP = lambda x: self.assertTrue(isinstance(x, np.ndarray))

        assertNaNs(gm.spacecraft_time)
        assertNaNs(gm.gps_time_status)
        assertNaNs(gm.gps_week)
        assertNaNs(gm.gps_milliseconds)
        assertNaNs(gm.position_status)
        assertNaNs(gm.position)
        assertNaNs(gm.position_sig)
        assertNaNs(gm.velocity_status)
        assertNaNs(gm.velocity)
        assertNaNs(gm.velocity_sig)
        assertNaNs(gm.v_latency)
        assertNaNs(gm.differential_age)
        assertNaNs(gm.solution_age)
        assertNaNs(gm.sats_tracked)
        assertNaNs(gm.sats_in_solution)

        assertNP(gm.position)
        assertNP(gm.position_sig)
        assertNP(gm.velocity)
        assertNP(gm.velocity_sig)

        self.assertEqual(len(gm.position), 3)
        self.assertEqual(len(gm.position_sig), 3)
        self.assertEqual(len(gm.velocity), 3)
        self.assertEqual(len(gm.velocity_sig), 3)

        self.assertEqual(len(gm.as_tuple), 15)

    def test_empty_msgpack(self):
        gm = messages.GPSMessage()
        b = gm.to_msgpack_b()
        self.assertIsInstance(b, bytes)

        gm2 = messages.GPSMessage(msgpack_b=b)

        self._verify_empty_message(gm2)


class TestOrbitEstimateMessage(unittest.TestCase):
    def test_empty(self):
        oem = messages.OrbitEstimateMessage()
        self._verify_empty_message(oem)

    def _verify_empty_message(self, oem):
        assertNaNs = lambda x: self.assertTrue(np.all(np.isnan(x)))
        assertNP = lambda x: self.assertTrue(isinstance(x, np.ndarray))

        assertNP(oem.spacecraft_time)
        assertNP(oem.state_estimate)
        assertNP(oem.disturbance_estimate)
        assertNP(oem.state_variance)
        assertNP(oem.disturbance_variance)

        self.assertEqual(len(oem.state_estimate), 6)
        self.assertEqual(len(oem.disturbance_estimate), 6)
        self.assertEqual(len(oem.state_variance), 6)
        self.assertEqual(len(oem.disturbance_variance), 6)

        assertNaNs(oem.spacecraft_time)
        assertNaNs(oem.state_estimate)
        assertNaNs(oem.disturbance_estimate)
        assertNaNs(oem.state_variance)
        assertNaNs(oem.disturbance_variance)

        self.assertEqual(len(oem.as_tuple), 6)

        sm_test = TestSensorMessage()
        sm_test._verify_empty_sensor_message(oem.sensor_message)

    def test_empty_msgpack(self):
        oem = messages.OrbitEstimateMessage()
        b = oem.to_msgpack_b()
        self.assertIsInstance(b, bytes)

        oem2 = messages.OrbitEstimateMessage(msgpack_b=b)

        self._verify_empty_message(oem2)

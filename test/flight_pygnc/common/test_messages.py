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

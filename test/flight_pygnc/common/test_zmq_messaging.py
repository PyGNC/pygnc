import numpy as np
import time
import unittest

from .. import context
from pygnc.common import messages  # type: ignore
from pygnc.common import zmq_messaging  # type: ignore

from .test_messages import (
    verify_empty_sensor_message,
    verify_empty_gps_message,
    verify_empty_orbit_estimate_message,
)


class TestZMQMessaging(unittest.TestCase):
    def test_empty_sensor_message(self):
        publisher_port = 6660
        synchronizer_port = 6670
        pub = zmq_messaging.zmqMessagePublisher(
            messages.SensorMessage,
            publisher_port=publisher_port,
            synchronizer_port=synchronizer_port,
            subscriber_names=[],
        )
        sub = zmq_messaging.zmqMessageSubscriber(
            messages.SensorMessage,
            "test_empty_sensor_message",
            publisher_port=publisher_port,
            synchronizer_port=synchronizer_port,
        )

        in_message = messages.SensorMessage()
        for _ in range(100):
            pub.send(in_message)
            out_message = sub.receive(block=False)
            if out_message is not None:
                break
            time.sleep(0.001)

        self.assertIsNotNone(out_message)
        self.assertIsInstance(out_message, messages.SensorMessage)
        verify_empty_sensor_message(self, out_message)

        del pub
        del sub

    def test_empty_gps_message(self):
        publisher_port = 6661
        synchronizer_port = 6671
        pub = zmq_messaging.zmqMessagePublisher(
            messages.GPSMessage,
            publisher_port=publisher_port,
            synchronizer_port=synchronizer_port,
            subscriber_names=[],
        )
        sub = zmq_messaging.zmqMessageSubscriber(
            messages.GPSMessage,
            "test_empty_gps_message",
            publisher_port=publisher_port,
            synchronizer_port=synchronizer_port,
        )

        in_message = messages.GPSMessage()
        for _ in range(100):
            pub.send(in_message)
            out_message = sub.receive(block=False)
            if out_message is not None:
                break
            time.sleep(0.001)

        self.assertIsNotNone(out_message)
        self.assertIsInstance(out_message, messages.GPSMessage)
        verify_empty_gps_message(self, out_message)

        del pub
        del sub

    def test_empty_orbit_estimate_message(self):
        publisher_port = None  # look up publisher_port using message type
        synchronizer_port = None
        pub = zmq_messaging.zmqMessagePublisher(
            messages.OrbitEstimateMessage,
            publisher_port=publisher_port,
            synchronizer_port=synchronizer_port,
            subscriber_names=[],
        )
        sub = zmq_messaging.zmqMessageSubscriber(
            messages.OrbitEstimateMessage,
            "test_empty_orbit_estimate_message",
            publisher_port=publisher_port,
            synchronizer_port=synchronizer_port,
        )

        in_message = messages.OrbitEstimateMessage()
        for _ in range(100):
            pub.send(in_message)
            out_message = sub.receive(block=False)
            if out_message is not None:
                break
            time.sleep(0.001)

        self.assertIsNotNone(out_message)
        self.assertIsInstance(out_message, messages.OrbitEstimateMessage)
        verify_empty_orbit_estimate_message(self, out_message)

        del pub
        del sub

    def test_invalid_message(self):
        publisher_port = 6663
        synchronizer_port = 6673
        pub = zmq_messaging.zmqMessagePublisher(
            messages.SensorMessage,
            publisher_port=publisher_port,
            synchronizer_port=synchronizer_port,
            subscriber_names=[],
        )
        sub = zmq_messaging.zmqMessageSubscriber(
            messages.SensorMessage,
            "test_invalid_message",
            publisher_port=publisher_port,
            synchronizer_port=synchronizer_port,
        )

        in_message = messages.GPSMessage()
        for _ in range(100):
            pub.send(in_message)
            out_message = sub.receive(block=False)
            if out_message is not None:
                break
            time.sleep(0.001)

        self.assertIsNone(out_message)
        self.assertNotIsInstance(out_message, messages.SensorMessage)

        del pub
        del sub

    def test_single_message(self):
        publisher_port = 6664
        synchronizer_port = 6674
        pub = zmq_messaging.zmqMessagePublisher(
            messages.SensorMessage,
            publisher_port=publisher_port,
            synchronizer_port=synchronizer_port,
            subscriber_names=[],
        )
        sub = zmq_messaging.zmqMessageSubscriber(
            messages.SensorMessage,
            "test_single_message",
            publisher_port=publisher_port,
            synchronizer_port=synchronizer_port,
        )

        spacecraft_time = 123456789e5
        mag_measurement = (100.0, 200.0, -300.3)
        raw_hall = 111.0
        gyro_measurement = (1e-4, 2e6, -123.0)
        sun_sensors = (1.0, 2.0, 3.0, 4.0, 5.0, 86000.0)

        in_message = messages.SensorMessage(
            spacecraft_time=spacecraft_time,
            mag_measurement=mag_measurement,
            raw_hall=raw_hall,
            gyro_measurement=gyro_measurement,
            sun_sensors=sun_sensors,
        )
        for _ in range(100):
            pub.send(in_message)
            out_message = sub.receive(block=False)
            if out_message is not None:
                break
            time.sleep(0.001)

        self.assertIsNotNone(out_message)
        self.assertIsInstance(out_message, messages.SensorMessage)

        np.testing.assert_array_almost_equal(
            np.array(spacecraft_time), out_message.spacecraft_time
        )
        np.testing.assert_array_almost_equal(
            np.array(mag_measurement), out_message.mag_measurement
        )
        np.testing.assert_array_almost_equal(np.array(raw_hall), out_message.raw_hall)
        np.testing.assert_array_almost_equal(
            np.array(gyro_measurement), out_message.gyro_measurement
        )
        np.testing.assert_array_almost_equal(
            np.array(sun_sensors), out_message.sun_sensors
        )

        del pub
        del sub

    def test_subscribed(self):
        publisher_port = 6665
        synchronizer_port = 6675
        subscriber_names = ["task_1", "task_2"]
        pub = zmq_messaging.zmqMessagePublisher(
            messages.SensorMessage,
            publisher_port=publisher_port,
            synchronizer_port=synchronizer_port,
            subscriber_names=subscriber_names,
        )
        self.assertFalse(pub._all_subscribed())
        pub._update_subscribed("task_1")
        self.assertFalse(pub._all_subscribed())
        pub._update_subscribed("fortytwo")
        self.assertFalse(pub._all_subscribed())
        pub._update_subscribed("task_2")
        self.assertTrue(pub._all_subscribed())

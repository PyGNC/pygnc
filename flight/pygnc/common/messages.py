"""
messages.py

Definitions of messages that are passed around on ZeroMQ.
Also acts as a common representation of data.
"""
import msgpack
import numpy as np


class MsgpackMessage:
    def to_msgpack_b(self):
        # requires child class to have implemented self.as_tuple
        return msgpack.packb(self.as_tuple)

    def _from_msgpack_b(self, msgpack_b):
        # requires child class to have implemented self.from_tuple(tup)
        self.from_tuple(msgpack.unpackb(msgpack_b))


class SensorMessage(MsgpackMessage):
    def __init__(
        self,
        spacecraft_time=np.nan,
        mag_measurement=(np.nan, np.nan, np.nan),
        raw_hall=np.nan,
        gyro_measurement=(np.nan, np.nan, np.nan),
        sun_sensors=(np.nan, np.nan, np.nan, np.nan, np.nan, np.nan),
        msgpack_b=None,
    ):
        if msgpack_b is not None:
            # initialize from msgpack data and ignore other entries
            super()._from_msgpack_b(msgpack_b)
        else:
            self._spacecraft_time = spacecraft_time
            self._mag_measurement = tuple(mag_measurement)
            self._raw_hall = raw_hall
            self._gyro_measurement = tuple(gyro_measurement)
            self._sun_sensors = tuple(sun_sensors)

    @property
    def as_tuple(self):
        return (
            self._spacecraft_time,
            self._mag_measurement,
            self._raw_hall,
            self._gyro_measurement,
            self._sun_sensors,
        )

    def from_tuple(self, tup):
        (
            self._spacecraft_time,
            self._mag_measurement,
            self._raw_hall,
            self._gyro_measurement,
            self._sun_sensors,
        ) = tup

    @property
    def spacecraft_time(self):
        return self._spacecraft_time

    @property
    def mag_measurement(self):
        return np.array(self._mag_measurement)

    @property
    def raw_hall(self):
        return self._raw_hall

    @property
    def gyro_measurement(self):
        return np.array(self._gyro_measurement)

    @property
    def sun_sensors(self):
        return np.array(self._sun_sensors)

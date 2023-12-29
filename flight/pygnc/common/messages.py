"""
messages.py

Definitions of messages that are passed around on ZeroMQ.
Also acts as a common representation of data.
"""
import brahe
import msgpack
import numpy as np


class MsgpackMessage:
    def to_msgpack_b(self):
        # requires child class to have implemented self.as_tuple
        return msgpack.packb(self.as_tuple)

    def _from_msgpack_b(self, msgpack_b):
        # requires child class to have implemented self._from_tuple(tup)
        return self._from_tuple(msgpack.unpackb(msgpack_b))

    def __repr__(self):
        return f"{self.__class__.__name__}: {self.as_tuple}"


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

    def _from_tuple(self, tup):
        (
            self._spacecraft_time,
            self._mag_measurement,
            self._raw_hall,
            self._gyro_measurement,
            self._sun_sensors,
        ) = tup
        return self

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


class GPSMessage(MsgpackMessage):
    def __init__(
        self,
        spacecraft_time=np.nan,
        gps_time_status=np.nan,
        gps_week=np.nan,
        gps_milliseconds=np.nan,
        position_status=np.nan,
        position_x=np.nan,
        position_y=np.nan,
        position_z=np.nan,
        position_sig_x=np.nan,
        position_sig_y=np.nan,
        position_sig_z=np.nan,
        velocity_status=np.nan,
        velocity_x=np.nan,
        velocity_y=np.nan,
        velocity_z=np.nan,
        velocity_sig_x=np.nan,
        velocity_sig_y=np.nan,
        velocity_sig_z=np.nan,
        v_latency=np.nan,
        differential_age=np.nan,
        solution_age=np.nan,
        sats_tracked=np.nan,
        sats_in_solution=np.nan,
        msgpack_b=None,
    ):
        if msgpack_b is not None:
            # initialize from msgpack data and ignore other entries
            super()._from_msgpack_b(msgpack_b)
        else:
            self._spacecraft_time = spacecraft_time
            self._gps_time_status = gps_time_status
            self._gps_week = gps_week
            self._gps_milliseconds = gps_milliseconds
            self._position_status = position_status
            self._position = (position_x, position_y, position_z)
            self._position_sig = (position_sig_x, position_sig_y, position_sig_z)
            self._velocity_status = velocity_status
            self._velocity = (velocity_x, velocity_y, velocity_z)
            self._velocity_sig = (velocity_sig_x, velocity_sig_y, velocity_sig_z)
            self._v_latency = v_latency
            self._differential_age = differential_age
            self._solution_age = solution_age
            self._sats_tracked = sats_tracked
            self._sats_in_solution = sats_in_solution

    @property
    def as_tuple(self):
        return (
            self._spacecraft_time,
            self._gps_time_status,
            self._gps_week,
            self._gps_milliseconds,
            self._position_status,
            self._position,
            self._position_sig,
            self._velocity_status,
            self._velocity,
            self._velocity_sig,
            self._v_latency,
            self._differential_age,
            self._solution_age,
            self._sats_tracked,
            self._sats_in_solution,
        )

    def _from_tuple(self, tup):
        (
            self._spacecraft_time,
            self._gps_time_status,
            self._gps_week,
            self._gps_milliseconds,
            self._position_status,
            self._position,
            self._position_sig,
            self._velocity_status,
            self._velocity,
            self._velocity_sig,
            self._v_latency,
            self._differential_age,
            self._solution_age,
            self._sats_tracked,
            self._sats_in_solution,
        ) = tup
        return self

    @property
    def spacecraft_time(self):
        return self._spacecraft_time

    @property
    def gps_time_status(self):
        return self._gps_time_status

    @property
    def gps_week(self):
        return self._gps_week

    @property
    def gps_milliseconds(self):
        return self._gps_milliseconds

    @property
    def position_status(self):
        return self._position_status

    @property
    def position(self):
        return np.array(self._position)

    @property
    def position_sig(self):
        return np.array(self._position_sig)

    @property
    def velocity_status(self):
        return self._velocity_status

    @property
    def velocity(self):
        return np.array(self._velocity)

    @property
    def velocity_sig(self):
        return np.array(self._velocity_sig)

    @property
    def v_latency(self):
        return self._v_latency

    @property
    def differential_age(self):
        return self._differential_age

    @property
    def solution_age(self):
        return self._solution_age

    @property
    def sats_tracked(self):
        return self._sats_tracked

    @property
    def sats_in_solution(self):
        return self._sats_in_solution


class SensorGPSMessage(MsgpackMessage):
    def __init__(
        self,
        sensor_message: SensorMessage = SensorMessage(),
        gps_message: GPSMessage = GPSMessage(),
        msgpack_b=None,
    ):
        super().__init__()
        if msgpack_b is not None:
            # initialize from msgpack data and ignore other entries
            super()._from_msgpack_b(msgpack_b)
        else:
            self._sensor_message = sensor_message
            self._gps_message = gps_message

    @property
    def as_tuple(self):
        return (
            self._sensor_message.as_tuple,
            self._gps_message.as_tuple,
        )

    def _from_tuple(self, tup):
        (
            sensor_message_tup,
            gps_message_tup,
        ) = tup
        self._sensor_message = SensorMessage()._from_tuple(sensor_message_tup)
        self._gps_message = GPSMessage()._from_tuple(gps_message_tup)
        return self

    @property
    def sensor_message(self):
        return self._sensor_message

    @property
    def gps_message(self):
        return self._gps_message


class OrbitEstimateMessage(MsgpackMessage):
    def __init__(
        self,
        epoch=brahe.epoch.Epoch("1000-01-01T00:00:00Z"),
        state_estimate=(np.nan, np.nan, np.nan, np.nan, np.nan, np.nan),
        disturbance_estimate=(np.nan, np.nan, np.nan, np.nan, np.nan, np.nan),
        state_variance=(np.nan, np.nan, np.nan, np.nan, np.nan, np.nan),
        disturbance_variance=(np.nan, np.nan, np.nan, np.nan, np.nan, np.nan),
        sensor_message: SensorMessage = SensorMessage(),
        msgpack_b=None,
    ):
        super().__init__()
        if msgpack_b is not None:
            # initialize from msgpack data and ignore other entries
            super()._from_msgpack_b(msgpack_b)
        else:
            self._epoch = epoch
            self._state_estimate = tuple(state_estimate)
            self._disturbance_estimate = tuple(disturbance_estimate)
            self._state_variance = tuple(state_variance)
            self._disturbance_variance = tuple(disturbance_variance)
            self._sensor_message = sensor_message

    @property
    def as_tuple(self):
        return (
            self._epoch.isoformat(),
            self._state_estimate,
            self._disturbance_estimate,
            self._state_variance,
            self._disturbance_variance,
            self._sensor_message.as_tuple,
        )

    def _from_tuple(self, tup):
        (
            epoch_iso,
            self._state_estimate,
            self._disturbance_estimate,
            self._state_variance,
            self._disturbance_variance,
            sensor_message_tup,
        ) = tup
        self._epoch = brahe.epoch.Epoch(epoch_iso)
        self._sensor_message = SensorMessage()._from_tuple(sensor_message_tup)
        return self

    @property
    def epoch(self):
        return self._epoch

    @property
    def state_estimate(self):
        return np.array(self._state_estimate)

    @property
    def disturbance_estimate(self):
        return np.array(self._disturbance_estimate)

    @property
    def state_variance(self):
        return np.array(self._state_variance)

    @property
    def disturbance_variance(self):
        return np.array(self._disturbance_variance)

    @property
    def sensor_message(self):
        return self._sensor_message
    
class RangingEstimateMessage(MsgpackMessage):
    def __init__(
        self,
        epoch=brahe.epoch.Epoch("1000-01-01T00:00:00Z"),
        state_estimate=(np.nan, np.nan, np.nan, np.nan, np.nan, np.nan),
        sensor_message: SensorMessage = SensorMessage(),
        msgpack_b=None,
    ):
        super().__init__()
        if msgpack_b is not None:
            # initialize from msgpack data and ignore other entries
            super()._from_msgpack_b(msgpack_b)
        else:
            self._epoch = epoch
            self._state_estimate = tuple(state_estimate)
            self._sensor_message = sensor_message

    @property
    def as_tuple(self):
        return (
            self._epoch.isoformat(),
            self._state_estimate,
            self._sensor_message.as_tuple,
        )

    def _from_tuple(self, tup):
        (
            epoch_iso,
            self._state_estimate,
            sensor_message_tup,
        ) = tup
        self._epoch = brahe.epoch.Epoch(epoch_iso)
        self._sensor_message = SensorMessage()._from_tuple(sensor_message_tup)
        return self

    @property
    def epoch(self):
        return self._epoch

    @property
    def state_estimate(self):
        return np.array(self._state_estimate)

    @property
    def sensor_message(self):
        return self._sensor_message


def _get_message_dict():
    """_get_message_dict()
    This bit of magic produces a dictionary that allows us to associate message class names with their types
    """
    all_classes = globals().values()
    message_dict = {
        f"{cls.__name__}": cls
        for cls in all_classes
        if isinstance(cls, type) and issubclass(cls, MsgpackMessage)
    }
    return message_dict


message_dict = _get_message_dict()

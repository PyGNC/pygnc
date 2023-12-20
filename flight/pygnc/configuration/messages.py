"""
Defines the ports that messages use.
"""

from ..common import messages

"""
message_port_dict provides the port number each message is communicated on.
These should be unique.
"""
message_configuration_dict = {
    messages.SensorGPSMessage.__name__: {
        "publisher_port": 5560,
        "synchronizer_port": 5561,
        "subscribers": ["telemetry_logger", "orbit_estimator"],
    },
    messages.OrbitEstimateMessage.__name__: {
        "publisher_port": 5570,
        "synchronizer_port": 5571,
        "subscribers": ["telemetry_logger", "attitude_estimator"],
    },
}

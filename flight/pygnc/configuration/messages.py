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
        "subscribers": [
            {"name": "telemetry_logger", "reply_port": 5561},
            {"name": "orbit_estimator", "reply_port": 5562},
        ],
    },
    messages.OrbitEstimateMessage.__name__: {
        "publisher_port": 5570,
        "subscribers": [
            {"name": "telemetry_logger", "reply_port": 5571},
            {"name": "attitude_estimator", "reply_port": 5572},
        ],
    },
}

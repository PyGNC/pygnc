"""
Defines the ports that messages use.
"""

from ..common import messages

"""
message_port_dict provides the port number each message is communicated on.
These should be unique.
"""
message_port_dict = {
        messages.SensorGPSMessage.__name__: 5560,
        messages.OrbitEstimateMessage.__name__: 5561,
    }
"""
this is not a unit test
it should be run from a standalone terminal with the working directory
pointing to the pygnc root directory (../../../ from here)

to run
python
>>> import test.flight_pygnc.common.verify_zmq_synchronize_pub.py
"""

import time

from ... import context
from pygnc.common import messages  # type: ignore
from pygnc.common import zmq_messaging  # type: ignore

publisher_port_1 = 6701
publisher_port_2 = 6702
publisher_port_3 = 6703
synchronizer_port_1 = 6711
synchronizer_port_2 = 6712
synchronizer_port_3 = 6713

pub_1 = zmq_messaging.zmqMessagePublisher(
    messages.SensorMessage,
    publisher_port=publisher_port_1,
    synchronizer_port=synchronizer_port_1,
    subscriber_names=["B_node2", "B_node3"],
)
pub_2 = zmq_messaging.zmqMessagePublisher(
    messages.GPSMessage,
    publisher_port=publisher_port_2,
    synchronizer_port=synchronizer_port_2,
    subscriber_names=["B_node2", "B_node3"],
)

zmq_messaging.zmq_synchronize(publishers=[pub_1, pub_2])

for i in range(10):
    pub_1.send(messages.SensorMessage(spacecraft_time=i))
    pub_2.send(messages.GPSMessage(spacecraft_time=i))
    time.sleep(0.1)

del pub_1
del pub_2

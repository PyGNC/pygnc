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

pub_3 = zmq_messaging.zmqMessagePublisher(
    messages.SensorGPSMessage,
    publisher_port=publisher_port_3,
    synchronizer_port=synchronizer_port_3,
    subscriber_names=["B_node3"],
)

sub_1 = zmq_messaging.zmqMessageSubscriber(
    messages.SensorMessage,
    "B_node2",
    publisher_port=publisher_port_1,
    synchronizer_port=synchronizer_port_1,
)

sub_2 = zmq_messaging.zmqMessageSubscriber(
    messages.GPSMessage,
    "B_node2",
    publisher_port=publisher_port_2,
    synchronizer_port=synchronizer_port_2,
)

zmq_messaging.zmq_synchronize(
    publishers=[
        pub_3,
    ],
    subscribers=[sub_1, sub_2],
)

for i in range(10):
    sm = sub_1.receive(block=True)
    gm = sub_2.receive(block=True)
    pub_3.send(
        messages.SensorGPSMessage(
            sensor_message=sm if sm is not None else messages.SensorMessage(),
            gps_message=gm if gm is not None else messages.GPSMessage(),
        )
    )
    time.sleep(0.1)

del pub_3
del sub_1
del sub_2

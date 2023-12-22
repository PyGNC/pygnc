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

sub_1 = zmq_messaging.zmqMessageSubscriber(
    messages.SensorMessage,
    "B_node3",
    publisher_port=publisher_port_1,
    synchronizer_port=synchronizer_port_1,
)

sub_2 = zmq_messaging.zmqMessageSubscriber(
    messages.GPSMessage,
    "B_node3",
    publisher_port=publisher_port_2,
    synchronizer_port=synchronizer_port_2,
)

sub_3 = zmq_messaging.zmqMessageSubscriber(
    messages.SensorGPSMessage,
    "B_node3",
    publisher_port=publisher_port_3,
    synchronizer_port=synchronizer_port_3,
)

zmq_messaging.zmq_synchronize(publishers=[], subscribers=[sub_1, sub_2, sub_3])

for i in range(10):
    sm = sub_1.receive(block=True)
    print(sm)
    gm = sub_2.receive(block=True)
    print(gm)
    sgm = sub_2.receive(block=True)
    print(sgm)
    time.sleep(0.1)

del sub_1
del sub_2
del sub_3

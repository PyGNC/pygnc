# this is not a unit test
import time

from .. import context
from pygnc.common import messages  # type: ignore
from pygnc.common import zmq_messaging  # type: ignore

publisher_port_1 = 6666
publisher_port_2 = 6667
synchronizer_port_1 = 6676
synchronizer_port_2 = 6677

sub_1 = zmq_messaging.zmqMessageSubscriber(
    messages.SensorMessage,
    "verify_synchronize_1",
    publisher_port=publisher_port_1,
    synchronizer_port=synchronizer_port_1,
)

sub_2 = zmq_messaging.zmqMessageSubscriber(
    messages.GPSMessage,
    "verify_synchronize_2",
    publisher_port=publisher_port_2,
    synchronizer_port=synchronizer_port_2,
)

zmq_messaging.zmq_synchronize(subscribers=[sub_1, sub_2])

for i in range(10):
    message = sub_1.receive(block=True)
    print(message)
    message = sub_2.receive(block=True)
    print(message)

del sub_1
del sub_2

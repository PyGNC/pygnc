"""
zmq_messaging.py

Abstract away zeromq messaging to a few easy to use functions built around the MsgpackMessage type.

A few architectural decisions were made here:
    - each task owns one zmqMessagePublisher for each message it publishes.
        - Each publisher has a single message that it publishes
    - each task has one zmqMessageSubscriber per message that it subscribes to
        - zmqMessageSubscriber.receive() can be blocking or non-blocking. Blocking allows tasks to
          be event based and run when they receive data.
"""

from . import messages
from ..configuration.messages import message_configuration_dict
import zmq


def _message_to_filter(message):
    return f"{message.__name__}::"

def _all_subscribed(subscribed):
    for _, value in subscribed.items():
        if not value:
            return False
    return True


class zmqMessagePublisher:
    """zmqMessagePublisher
    message_type: the type of message (inherits from MsgpackMessage) this publishes.
        The port is determined from this message unless explicitly specified with the port argument.
        Ports should always be determined from the message (via message_port_dict).
        The port argument is only for unit test fixtures where tests are run in parallel and port conflicts can occur.
    """

    def __init__(self, message_type, publisher_port=None, synchronizer_port=None):
        self.message_type = message_type
        if publisher_port is None:
            publisher_port = message_configuration_dict[message_type.__name__]["publisher_port"]
        self.context = zmq.Context()
        self.publisher = self.context.socket(zmq.PUB)
        self.publisher.bind(f"tcp://*:{publisher_port}")

        synchronizer = self.context.socket(zmq.REP)
        synchronizer.bind(f"tcp://*:{synchronizer_port}")

        subscribers = message_configuration_dict[message_type.__name__]["subscribers"]
        subscribed = dict()
        for subscriber in subscribers:
            subscribed[subscriber["name"]] = False
        
        while not _all_subscribed(subscribed):
            pass
            # send synch message

            # check for response from subscribers
            
         


    def __del__(self):
        self.publisher.close()
        self.context.term()

    def send(self, message: messages.MsgpackMessage):
        message_filter = _message_to_filter(message.__class__)
        self.publisher.send(message_filter.encode("utf-8") + message.to_msgpack_b())


class zmqMessageSubscriber:
    """
    message_type: a message type that inherits from MsgpackMessage
        The port is determined from this message unless explicitly specified with the port argument.
        Ports should always be determined from the message (via message_port_dict).
        The port argument is only for unit test fixtures where tests are run in parallel and port conflicts can occur.
    return_latest: if true, set the CONFLATE flag so old messages are dropped and the latest one is returned. 
        For a real time GNC system this is typically what we want.
        For a batch estimator it is better for it to be false and for the messages to queue.
    """

    def __init__(
        self, message_type: messages.MsgpackMessage, port=None, return_latest=True
    ):
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)

        # need to set socket options before connecting
        if return_latest:
            self.subscriber.setsockopt(zmq.CONFLATE, 1)

        self.message_filter = _message_to_filter(message_type)
        self.message_type = message_type
        if port is None:
            port = message_configuration_dict[message_type.__name__]["publisher_port"]

        self.subscriber.setsockopt_string(
            zmq.SUBSCRIBE, ""
        )  # subscribe to everything on this port

        self.subscriber.connect(f"tcp://localhost:{port}")

    def receive(self, block=True):

        block_flag = 0 if block else zmq.NOBLOCK

        try:
            raw_message = self.subscriber.recv(flags=block_flag)
        except zmq.ZMQError as e:
            if str(e) == "Resource temporarily unavailable":
                # ZMQError was raised because no message is available
                return None
            else:
                raise e
        filter_len = len(self.message_filter)
        try:
            recieved_filter = raw_message[:filter_len].decode("utf-8")
            if raw_message[:filter_len].decode("utf-8") == self.message_filter:
                return self.message_type(msgpack_b=raw_message[filter_len:])
        except UnicodeDecodeError:
            # filter string didn't decode
            pass

        return None

"""
zmq_messaging.py

Abstract away zeromq messaging to a few easy to use functions built around the MsgpackMessage type.

A few architectural decisions were made here:
    - each task has one zmqMessagePublisher that it owns.
        - Each publisher has a single message that it publishes
    - each task has one zmqMessageSubscriber that it owns
"""

from . import messages
from ..configuration.messages import message_port_dict
import zmq


def _message_to_filter(message):
    return f"{message.__name__}::"


class zmqMessagePublisher:
    """zmqMessagePublisher
    port: the port number to start this publisher on.
    """

    def __init__(self, message_type):
        self.message_type == message_type
        port = message_port_dict[message_type.__name__]
        self.context = zmq.Context()
        self.publisher = self.context.socket(zmq.PUB)
        self.publisher.bind(f"tcp://*:{port}")

    def __del__(self):
        self.publisher.close()
        self.context.term()

    def send(self, message: messages.MsgpackMessage):
        message_filter = _message_to_filter(message.__class__)
        self.publisher.send(message_filter.encode("utf-8") + message.to_msgpack_b())


class zmqMessageSubscriber:
    """
    message_type: a message type that inherits from MsgpackMessage
        this is the messages that the listener will subscribe to
    return_latest: if true, set the CONFLATE flag so old messages are dropped and the latest one is returned
    """

    def __init__(self, message_type: messages.MsgpackMessage, return_latest=True):
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)

        # need to set socket options before connecting
        if return_latest:
            self.subscriber.setsockopt(zmq.CONFLATE, 1)

        self.message_filter = _message_to_filter(message_type)
        self.message_type = message_type
        port = message_port_dict[message_type.__name__]

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

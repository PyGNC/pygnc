"""
zmq_messaging.py

Abstract away zeromq messaging to a few easy to use functions built around the MsgpackMessage type.

A few architectural decisions were made here:
    - each task has one zmqMessagePublisher that it owns.
        - Each publisher has a single message that it publishes
    - each task has one zmqMessageSubscriber that it owns
"""

from . import messages
import zmq

def _message_name_to_filter(message_name):
    return f"{message_name}::"

class zmqMessagePublisher:
    """ zmqMessagePublisher
    port: the port number to start this publisher on.
    """
    def __init__(self, port):
        self.context = zmq.Context()
        self.publisher = self.context.socket(zmq.PUB)
        self.publisher.bind(f"tcp://*:{port}")

    def __del__(self):
        self.publisher.close()
        self.context.term()
    
    def send(self, message:messages.MsgpackMessage):
        message_filter = _message_name_to_filter(message.__class__.__name__)
        self.publisher.send(message_filter.encode("utf-8")+message.to_msgpack_b())


class zmqMessageSubscriber:
    """
        port: the port number to start this subscriber on.
        message_list: a list of message types that inherit from MsgpackMessage
            these are the messages that the listener will subscribe to
        return_latest: if true, set the CONFLATE flag so old messages are dropped and the latest one is returned
    """
    def __init__(self, port, message_list, return_latest=True):
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)

        # need to set socket options before connecting
        if return_latest:
            self.subscriber.setsockopt(zmq.CONFLATE, 1)
        
        self.message_dict = {}
        if message_list is not None:
            for message_type in message_list:
                message_filter = _message_name_to_filter(message_type.__name__)
                self.subscriber.setsockopt_string(zmq.SUBSCRIBE, message_filter) # each argument defines a filter value
                self.message_dict[message_filter] = message_type
        else:
            self.subscriber.setsockopt_string(zmq.SUBSCRIBE, "") # subscribe to all

        self.subscriber.connect(f"tcp://localhost:{port}")
    
    def recieve(self, block=True):

        block_flag = 0 if block else zmq.NOBLOCK

        try:
            raw_message = self.subscriber.recv(flags=block_flag)
        except zmq.ZMQError as e:
            # ZMQError was raised because no message is available
            return None

        # message received, convert to appropriate type
        for message_filter, message_type in self.message_dict.items():
            filter_len = len(message_filter)
            if raw_message[:filter_len].decode('utf-8') == message_filter:
                return message_type(msgpack_b=raw_message[filter_len:])
        
        # no matching filter - this shouldn't happen
        raise KeyError(f"Message doesn't match any filters. Message: {raw_message}")

        return None


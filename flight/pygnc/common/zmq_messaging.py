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

_SYNC_WORD = "SYNC::".encode("utf-8")


def _message_to_filter(message):
    return f"{message.__name__}::"


def _recv(socket, block=False):
    block_flag = 0 if block else zmq.NOBLOCK
    try:
        raw_message = socket.recv(flags=block_flag)
        return raw_message
    except zmq.ZMQError as e:
        if str(e) == "Resource temporarily unavailable":
            # ZMQError was raised because no message is available
            return None
        else:
            raise e


def zmq_synchronize(publishers, subscribers):
    """
    This performs synchronization between the publishers and subscribers belonging to a task.
    All publishers and all subscribers belonging to the task should be provided as inputs.
    """
    sync_count = 0
    while True:
        all_synchronized = True

        for publisher in publishers:
            all_synchronized &= publisher.synchronizer_update()

        for subscriber in subscribers:
            subscriber.synchronizer_update()

        if all_synchronized:
            print(f"Synchronized after {sync_count} iterations")
            break
        sync_count += 1


class zmqMessagePublisher:
    """zmqMessagePublisher
    message_type: the type of message (inherits from MsgpackMessage) this publishes.
        The port is determined from this message unless explicitly specified with the port argument.
        Ports should always be determined from the message (via message_port_dict).
        The port argument is only for unit test fixtures where tests are run in parallel and port conflicts can occur.
    """

    def __init__(
        self,
        message_type,
        publisher_port=None,
        synchronizer_port=None,
        subscriber_names=None,
    ):
        self.message_type = message_type
        self.context = zmq.Context()

        if publisher_port is None:
            publisher_port = message_configuration_dict[message_type.__name__][
                "publisher_port"
            ]
        self.publisher = self.context.socket(zmq.PUB)
        self.publisher.bind(f"tcp://*:{publisher_port}")

        if synchronizer_port is None:
            synchronizer_port = message_configuration_dict[message_type.__name__][
                "synchronizer_port"
            ]
        self.synchronizer = self.context.socket(zmq.REP)
        self.synchronizer.bind(f"tcp://*:{synchronizer_port}")

        if subscriber_names is None:
            subscriber_names = message_configuration_dict[message_type.__name__][
                "subscribers"
            ]
        self.subscribed = {
            subscriber_name: False for subscriber_name in subscriber_names
        }

    def __del__(self):
        self.publisher.close()
        self.synchronizer.close()
        self.context.term()

    def send(self, message: messages.MsgpackMessage):
        message_filter = _message_to_filter(message.__class__)
        self.publisher.send(message_filter.encode("utf-8") + message.to_msgpack_b())

    def _update_subscribed(self, subscriber_name):
        if subscriber_name in self.subscribed:
            self.subscribed[subscriber_name] = True

    def _all_subscribed(self):
        for _, value in self.subscribed.items():
            if not value:
                return False
        return True

    def synchronizer_update(self):
        """
        synchronization is expected to work as follows:
         - the publisher sends the sync message on the pub-sub connection
         - when a subscriber recieves the sync message it responds with
            its task name on the req-rep connection
         - when the publisher gets the task name, it marks that task as sync'd in self.subscribed
         - once all subscribers report that they are getting messages, the connection is sync'd
        """
        self.publisher.send()
        raw_message = _recv(self.synchronizer, block=False)
        if raw_message is not None:
            message = raw_message.decode("utf-8")
            self._update_subscribed(message)

        return self._all_subscribed()


class zmqMessageSubscriber:
    """
    message_type: a message type that inherits from MsgpackMessage
        The port is determined from this message unless explicitly specified with the port argument.
        Ports should always be determined from the message (via message_port_dict).
        The port argument is only for unit test fixtures where tests are run in parallel and port conflicts can occur.
    task_name: the name of the task this subscriber belongs to. Should match the subscriber names in configuration/messages.py
    return_latest: if true, set the CONFLATE flag so old messages are dropped and the latest one is returned.
        For a real time GNC system this is typically what we want.
        For a batch estimator it is better for it to be false and for the messages to queue.
    """

    def __init__(
        self,
        message_type: messages.MsgpackMessage,
        task_name,
        return_latest=True,
        publisher_port=None,
        synchronizer_port=None,
    ):
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)

        # need to set socket options before connecting
        if return_latest:
            self.subscriber.setsockopt(zmq.CONFLATE, 1)

        self.message_filter = _message_to_filter(message_type)
        self.message_type = message_type
        if publisher_port is None:
            publisher_port = message_configuration_dict[message_type.__name__][
                "publisher_port"
            ]

        self.subscriber.setsockopt_string(
            zmq.SUBSCRIBE, ""
        )  # subscribe to everything on this port

        self.subscriber.connect(f"tcp://localhost:{publisher_port}")

        if synchronizer_port is None:
            synchronizer_port = message_configuration_dict[message_type.__name__][
                "synchronizer_port"
            ]
        self.synchronizer = self.context.socket(zmq.REQ)
        self.synchronizer.connect(f"tcp://localhost:{synchronizer_port}")

        self.task_name = task_name

    def receive(self, block=True):

        raw_message = _recv(self.subscriber, block=block)

        if raw_message is not None:
            filter_len = len(self.message_filter)
            try:
                recieved_filter = raw_message[:filter_len].decode("utf-8")
                if raw_message[:filter_len].decode("utf-8") == self.message_filter:
                    return self.message_type(msgpack_b=raw_message[filter_len:])
            except UnicodeDecodeError:
                # filter string didn't decode
                pass

        return None

    def synchronizer_update(self):
        raw_message = _recv(self.subscriber, block=False)
        if raw_message is not None and raw_message == _SYNC_WORD:
            self.synchronizer.send(self.task_name.encode("utf-8"))
            return True
        else:
            return False

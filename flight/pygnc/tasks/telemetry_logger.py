from ..common.zmq_messaging import zmqMessageSubscriber
from ..common import messages


def main():
    oem_sub = zmqMessageSubscriber(messages.OrbitEstimateMessage, return_latest=True)
    while True:
        message = oem_sub.receive(block=True)
        print(message)


if __name__ == "__main__":
    main()

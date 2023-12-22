import os
import numpy as np

from ..common.zmq_messaging import zmqMessageSubscriber
from ..common import messages
from ..configuration import pygnc as pygnc_config


def update_status_file(
    oem_message: messages.OrbitEstimateMessage,
):
    with open(
        pygnc_config.status_filepath, "wb"
    ) as status_file:  # overwrite status file every time
        status_file.write(oem_message.to_msgpack_b())


def update_info_file(
    oem_message: messages.OrbitEstimateMessage,
):
    with open(
        pygnc_config.info_filepath, "w"
    ) as info_file:  # overwrite info file every time
        info_file.write(
            f"""
T:{str(oem_message.epoch)}
OE:{np.array2string(oem_message.state_estimate, max_line_width=1000, precision=4, separator=',')}
GY:{np.array2string(oem_message.sensor_message.gyro_measurement, max_line_width=1000, precision=4, separator=',')}
MG:{np.array2string(oem_message.sensor_message.mag_measurement, max_line_width=1000, precision=4, separator=',')}
"""
        )


def update_log_file(start_epoch, message: messages.MsgpackMessage):
    # unique log per message, logs start fresh when pygnc is started
    log_filepath = os.path.join(
        pygnc_config.log_dir_filepath,
        message.__class__.__name__,
        f"log_{str(start_epoch)}.bin",
    )
    with open(log_filepath, "ab") as log_file:
        log_file.write(message.to_msgpack_b())


def main():
    oem_sub = zmqMessageSubscriber(messages.OrbitEstimateMessage, return_latest=True)
    start_epoch = None
    os.makedirs(pygnc_config.log_dir_filepath, exist_ok=True)
    os.makedirs(
        os.path.join(
            pygnc_config.log_dir_filepath, messages.OrbitEstimateMessage.__name__
        ),
        exist_ok=True,
    )
    while True:
        oem_message = oem_sub.receive(block=True)
        if oem_message is not None:
            update_info_file(oem_message)
            update_status_file(oem_message)
            if start_epoch is None:
                start_epoch = oem_message.epoch
            update_log_file(start_epoch, oem_message)


if __name__ == "__main__":
    main()

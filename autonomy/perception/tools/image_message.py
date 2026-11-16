"""Read AutoMsgs image buffers and source frame identifiers."""

import numpy as np


def frame_key(message):
    header = message.header
    stamp = header.stamp
    if (not header.frame_id or stamp.sec < 0 or
            not 0 <= stamp.nanosec < 1_000_000_000 or
            (stamp.sec == 0 and stamp.nanosec == 0)):
        return None
    return header.frame_id, stamp.sec, stamp.nanosec


def image_array(message, dtype, channels):
    dtype = np.dtype(dtype)
    row_bytes = message.width * channels * dtype.itemsize
    if (not message.width or not message.height or message.step < row_bytes
            or len(message.data) != message.step * message.height):
        raise ValueError("Invalid image dimensions, stride or buffer size.")
    pixels = np.ndarray((message.height, message.width, channels),
                        dtype=dtype, buffer=message.data,
                        strides=(message.step, channels * dtype.itemsize,
                                 dtype.itemsize))
    return pixels

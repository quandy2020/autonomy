#!/usr/bin/env python3
"""Complete registered RGB-D depth with a two-input ONNX export."""

import argparse
import logging
from pathlib import Path
from threading import Lock

import numpy as np

from image_message import frame_key, image_array
from demo_config import parse_demo_arguments


class DepthCompletion:
    def __init__(self, model, provider, min_depth, max_depth):
        import onnxruntime as ort

        if provider not in ort.get_available_providers():
            raise ValueError(f"Unavailable ONNX Runtime provider: {provider}")
        self.session = ort.InferenceSession(str(model), providers=[provider])
        inputs = {item.name: item for item in self.session.get_inputs()}
        outputs = {item.name: item for item in self.session.get_outputs()}
        if (set(inputs) != {"rgb", "sparse_depth"} or
                set(outputs) != {"depth"} or
                any(item.type != "tensor(float)" or len(item.shape) != 4
                    for item in (*inputs.values(), *outputs.values()))):
            raise ValueError("Expected float32 NCHW rgb, sparse_depth and depth tensors.")
        self.minimum = min_depth
        self.maximum = max_depth

    def run(self, rgb, depth):
        measured = np.isfinite(depth) & (depth >= self.minimum) & (depth <= self.maximum)
        sparse = np.where(measured, depth, 0).astype(np.float32)
        image = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32)
        image /= 255
        predicted, = self.session.run(
            ["depth"], {"rgb": image, "sparse_depth": sparse[None, None]})
        if predicted.shape != (1, 1, *depth.shape):
            raise ValueError("Completed depth must match the source resolution.")
        predicted = predicted[0, 0]
        valid = (np.isfinite(predicted) & (predicted >= self.minimum) &
                 (predicted <= self.maximum))
        # Learned values fill holes only. The calibrated measurement wins.
        return np.where(measured, depth, np.where(valid, predicted, np.nan))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--provider", default="CPUExecutionProvider")
    parser.add_argument("--rgb-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic")
    parser.add_argument("--output-topic", default="/perception/depth/completed")
    parser.add_argument("--min-depth", type=float, default=0.15)
    parser.add_argument("--max-depth", type=float, default=10.0)
    args = parse_demo_arguments(
        parser, "depth_completion_demo", required=("model", "depth_topic"))
    topics = (args.rgb_topic, args.depth_topic, args.output_topic)
    if (any(not topic.startswith("/") for topic in topics) or
            len(set(topics)) != len(topics) or
            not 0 < args.min_depth < args.max_depth < float("inf")):
        parser.error("Expected distinct absolute topics and a finite depth range.")

    import autolink
    from automsgs.msgs.sensor_msgs.image_pb2 import Image

    completion = DepthCompletion(args.model, args.provider, args.min_depth, args.max_depth)
    lock = Lock()
    latest = {}
    autolink.init("perception_depth_completion")
    try:
        node = autolink.Node("perception_depth_completion")
        writer = node.create_writer(args.output_topic, Image, qos_depth=1)

        def receive(kind, message):
            with lock:
                latest[kind] = message
                rgb = latest.get("rgb")
                depth = latest.get("depth")
                if rgb is None or depth is None:
                    return
                key = frame_key(rgb)
                if key is None or key != frame_key(depth):
                    return
                latest.clear()
                try:
                    if (rgb.encoding not in ("rgb8", "bgr8") or depth.encoding != "32FC1"
                            or (rgb.width, rgb.height) != (depth.width, depth.height)):
                        raise ValueError("Expected registered RGB and 32FC1 depth.")
                    color = image_array(rgb, "u1", 3)
                    if rgb.encoding == "bgr8":
                        color = color[..., ::-1]
                    meters = image_array(
                        depth, ">f4" if depth.is_bigendian else "<f4", 1)[..., 0]
                    completed = completion.run(color, meters)
                    output = Image(width=depth.width, height=depth.height,
                                   step=4 * depth.width, encoding="32FC1",
                                   data=completed.astype("<f4").tobytes())
                    output.header.CopyFrom(depth.header)
                    writer.write(output)
                except (ValueError, RuntimeError) as error:
                    logging.warning("Depth completion skipped: %s", error)

        node.create_reader(args.rgb_topic, lambda msg: receive("rgb", msg), data_type=Image)
        node.create_reader(args.depth_topic, lambda msg: receive("depth", msg), data_type=Image)
        node.spin()
    finally:
        autolink.shutdown()


if __name__ == "__main__":
    main()

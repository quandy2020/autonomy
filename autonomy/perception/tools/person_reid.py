#!/usr/bin/env python3
"""Associate person tracks with an ONNX appearance embedding."""

import argparse
import logging
from pathlib import Path
from threading import Lock
from uuid import uuid4

import numpy as np
from PIL import Image as PillowImage

from image_message import frame_key, image_array
from demo_config import parse_demo_arguments


class AppearanceTracks:
    def __init__(self, similarity, margin, lifetime):
        self.similarity = similarity
        self.margin = margin
        self.lifetime = lifetime * 1_000_000_000
        self.tracks = {}
        self.last_stamp = None
        self.frame = None

    def update(self, frame, stamp, embeddings):
        if self.frame != frame or (self.last_stamp is not None and stamp < self.last_stamp):
            self.tracks.clear()
        self.frame = frame
        self.last_stamp = stamp
        self.tracks = {key: entry for key, entry in self.tracks.items()
                       if stamp - entry[0] <= self.lifetime}
        keys = list(self.tracks)
        matches = []
        for vector in embeddings:
            scores = sorted(((float(vector @ self.tracks[key][1]), key)
                             for key in keys), reverse=True)
            key = None
            if (scores and scores[0][0] >= self.similarity and
                    (len(scores) == 1 or scores[0][0] - scores[1][0] >= self.margin)):
                key = scores[0][1]
            matches.append(key)
        claims = {}
        for key in matches:
            claims[key] = claims.get(key, 0) + 1
        ids = []
        for key, vector in zip(matches, embeddings):
            if key is None or claims[key] != 1:
                key = uuid4().hex
            self.tracks[key] = (stamp, vector)
            ids.append(key)
        return ids


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--provider", default="CPUExecutionProvider")
    parser.add_argument("--rgb-topic", default="/camera/color/image_raw")
    parser.add_argument("--tracks-topic", default="/perception/base/tracks")
    parser.add_argument("--output-topic", default="/perception/reid/tracks")
    parser.add_argument("--similarity", type=float, default=0.82)
    parser.add_argument("--margin", type=float, default=0.08)
    parser.add_argument("--lifetime", type=float, default=1.5)
    args = parse_demo_arguments(
        parser, "person_reid_demo", required=("model",))
    topics = (args.rgb_topic, args.tracks_topic, args.output_topic)
    if (any(not topic.startswith("/") for topic in topics) or
            len(set(topics)) != len(topics) or not 0 < args.similarity <= 1 or
            not 0 < args.margin <= 1 or not 0 < args.lifetime < float("inf")):
        parser.error("Invalid topics or appearance matching parameters.")

    import autolink
    import onnxruntime as ort
    from automsgs.msgs.sensor_msgs.image_pb2 import Image
    from automsgs.msgs.vision_msgs.detection2d_array_pb2 import Detection2DArray

    if args.provider not in ort.get_available_providers():
        parser.error(f"Unavailable ONNX Runtime provider: {args.provider}")
    session = ort.InferenceSession(str(args.model), providers=[args.provider])
    inputs, outputs = session.get_inputs(), session.get_outputs()
    if (len(inputs) != 1 or len(outputs) != 1 or
            inputs[0].type != "tensor(float)" or outputs[0].type != "tensor(float)" or
            inputs[0].shape != [1, 3, 256, 128] or len(outputs[0].shape) != 2):
        parser.error("Expected float32 [1,3,256,128] input and [1,D] embedding.")

    association = AppearanceTracks(args.similarity, args.margin, args.lifetime)
    lock = Lock()
    frames = {"rgb": {}, "tracks": {}}
    autolink.init("perception_person_reid")
    try:
        node = autolink.Node("perception_person_reid")
        writer = node.create_writer(args.output_topic, Detection2DArray, qos_depth=1)

        def receive(kind, message):
            key = frame_key(message)
            if key is None:
                return
            with lock:
                bucket = frames[kind]
                bucket[key] = message
                if len(bucket) > 4:
                    del bucket[next(iter(bucket))]
                if key not in frames["rgb"] or key not in frames["tracks"]:
                    return
                rgb = frames["rgb"].pop(key)
                tracks = frames["tracks"].pop(key)
                try:
                    if rgb.encoding not in ("rgb8", "bgr8"):
                        raise ValueError("Expected rgb8 or bgr8.")
                    pixels = image_array(rgb, "u1", 3)
                    if rgb.encoding == "bgr8":
                        pixels = pixels[..., ::-1]
                    embeddings = []
                    output = Detection2DArray()
                    output.header.CopyFrom(tracks.header)
                    for detection in tracks.detections:
                        box = detection.bbox
                        x, y = box.center.position.x, box.center.position.y
                        w, h = box.size_x, box.size_y
                        if not np.isfinite([x, y, w, h]).all() or min(w, h) <= 0:
                            continue
                        x0, x1 = np.clip([x - w / 2, np.ceil(x + w / 2)], 0, rgb.width).astype(int)
                        y0, y1 = np.clip([y - h / 2, np.ceil(y + h / 2)], 0, rgb.height).astype(int)
                        if x1 <= x0 or y1 <= y0:
                            continue
                        patch = PillowImage.fromarray(pixels[y0:y1, x0:x1]).resize(
                            (128, 256), PillowImage.Resampling.BILINEAR)
                        tensor = np.asarray(patch, dtype=np.float32) / 255
                        tensor -= np.array([0.485, 0.456, 0.406], dtype=np.float32)
                        tensor /= np.array([0.229, 0.224, 0.225], dtype=np.float32)
                        values, = session.run(None, {
                            inputs[0].name: np.ascontiguousarray(tensor.transpose(2, 0, 1)[None])})
                        if values.ndim != 2 or values.shape[0] != 1 or not values.shape[1]:
                            raise ValueError("Invalid appearance embedding shape.")
                        norm = np.linalg.norm(values[0])
                        if not np.isfinite(norm) or norm <= 0:
                            continue
                        embeddings.append(values[0] / norm)
                        output.detections.add().CopyFrom(detection)
                    for detection, track_id in zip(
                            output.detections, association.update(
                                key[0], key[1] * 1_000_000_000 + key[2], embeddings)):
                        detection.id = track_id
                    writer.write(output)
                except Exception:
                    logging.exception("Person ReID frame failed.")

        node.create_reader(args.rgb_topic, lambda msg: receive("rgb", msg), data_type=Image)
        node.create_reader(args.tracks_topic, lambda msg: receive("tracks", msg),
                           data_type=Detection2DArray)
        node.spin()
    finally:
        autolink.shutdown()


if __name__ == "__main__":
    main()

"""Regression for configuration types rejected by the real Atlas/OpenCV loader."""

from pathlib import Path
import sys
import unittest

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from validate_rgbd_localization import camera_settings, sensor_height, verify_rendered_camera_height
from automsgs.msgs.sensor_msgs.camera_info_pb2 import CameraInfo
from automsgs.msgs.sensor_msgs.image_pb2 import Image
from automsgs.msgs.tf2_msgs.tf_message_pb2 import TFMessage


class RgbdConfigurationTest(unittest.TestCase):
    def setUp(self):
        self.info = CameraInfo(width=640, height=480)
        self.info.header.frame_id = 'camera_optical_frame'
        self.info.k.extend([320, 0, 320, 0, 320, 240, 0, 0, 1])
        self.rgb = Image(width=640, height=480, encoding='rgb8')
        self.depth = Image(width=640, height=480, encoding='32FC1')
        self.rgb.header.frame_id = self.info.header.frame_id
        self.depth.header.frame_id = self.info.header.frame_id

    def test_opencv_sees_real_intrinsics_and_integer_fps(self):
        source = camera_settings(self.info, self.rgb, self.depth, 10.0, .21)
        storage = cv2.FileStorage(source, cv2.FILE_STORAGE_READ | cv2.FILE_STORAGE_MEMORY)
        try:
            self.assertTrue(storage.isOpened())
            for key in ('Camera1.fx', 'Camera1.fy', 'Camera1.cx', 'Camera1.cy',
                        'RGBD.DepthMapFactor', 'Atlas.Map.SensorHeight',
                        'Viewer.KeyFrameSize', 'Viewer.KeyFrameLineWidth', 'Viewer.GraphLineWidth',
                        'Viewer.PointSize', 'Viewer.CameraSize', 'Viewer.CameraLineWidth',
                        'Viewer.ViewpointX', 'Viewer.ViewpointY', 'Viewer.ViewpointZ', 'Viewer.ViewpointF'):
                self.assertTrue(storage.getNode(key).isReal(), key)
            for key in ('Camera.width', 'Camera.height', 'Camera.fps', 'ORBextractor.nFeatures'):
                self.assertTrue(storage.getNode(key).isInt(), key)
            self.assertEqual(storage.getNode('RGBD.DepthMapFactor').real(), 1.0)
            self.assertEqual(storage.getNode('Camera1.cx').real(), 320)
            self.assertEqual(storage.getNode('Camera.fps').real(), 10)
        finally:
            storage.release()

    def test_unregistered_and_nonmetric_images_are_rejected(self):
        for encoding, width in [('16UC1', 640), ('32FC1', 320)]:
            self.depth.encoding, self.depth.width = encoding, width
            with self.assertRaises(ValueError):
                camera_settings(self.info, self.rgb, self.depth, 10, .21)

    def test_distortion_and_fractional_fps_are_rejected(self):
        self.info.d.append(.1)
        with self.assertRaises(ValueError):
            camera_settings(self.info, self.rgb, self.depth, 10, .21)
        del self.info.d[:]
        with self.assertRaises(ValueError):
            camera_settings(self.info, self.rgb, self.depth, 10.5, .21)

    def test_rendering_origin_must_match_actual_installation(self):
        verify_rendered_camera_height(.21, (0., .21, -.05))
        for actual in [(0., .6, -.05), (0., float('nan'), 0.), (0., .21)]:
            with self.assertRaises(ValueError):
                verify_rendered_camera_height(.21, actual)

    def test_height_comes_from_the_complete_static_chain(self):
        transforms = TFMessage()
        for parent, child, height in [('base_footprint', 'base_link', .01),
                                      ('base_link', 'camera_optical_frame', .2)]:
            transform = transforms.transforms.add()
            transform.header.frame_id, transform.child_frame_id = parent, child
            transform.transform.translation.z = height
            transform.transform.rotation.w = 1
        self.assertAlmostEqual(sensor_height(transforms, 'base_footprint', self.info.header.frame_id), .21)
        with self.assertRaises(ValueError):
            sensor_height(transforms, 'unconnected_base', self.info.header.frame_id)


if __name__ == '__main__':
    unittest.main()

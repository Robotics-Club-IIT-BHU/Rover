#!/usr/bin/env python3
"""
panel_camera.py, a copy of the colour camera small enough for the SSH tunnel.

The raw colour stream is 640x480 rgb8, 921,672 bytes a frame at 15 Hz, which
is 13.8 MB/s.  foxglove_bridge's WebSocket server forwards sensor_msgs/Image
unchanged.  It does NOT transcode to H.264: that only happens in its separate
remote-access (WebRTC) gateway, which needs a Foxglove device token and is
off here.  Measured 2026-10-02, the SSH tunnel over campus WiFi carried 4.7 to
7.0 MB/s, so the raw image alone was 2-3x what the link could take.  Every
queue between the bridge and the laptop filled up (bridge TCP send buffer
4 MB, sshd receive buffer 6 MB, SSH socket about 2 MB), and the costmap, TF
and everything else waited behind seconds of video in the same TCP stream.

This node publishes /athena/camera/compressed (sensor_msgs/CompressedImage,
JPEG), throttled and optionally downscaled.  640x480 at quality 70 measured
28 KiB a frame indoors (expect up to about twice that outdoors, where grass
and foliage compress worse), so 5 Hz is 0.15-0.3 MB/s instead of 13.8 MB/s.
Foxglove's Image panel decodes it natively.

It costs nothing while nobody is watching.  It subscribes to the raw image
only while something subscribes to the output (checked once a second), so
with Foxglove closed, or the Image panel removed, it holds no subscription and
does no work.  Frames it skips are dropped as raw bytes, never deserialized.
The encoding happens here, not in the camera driver, so the frames
rgbd_odometry receives are unchanged and are not delayed.
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.serialization import deserialize_message

from sensor_msgs.msg import CompressedImage, Image

# Best effort, depth 1: only the newest frame matters, and a RELIABLE reader
# would make the camera driver hold and repair 900 KB frames on our behalf.
RAW_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
OUT_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)

# encoding -> (channels, cv2 conversion to BGR, or None if already BGR/mono)
ENCODINGS = {
    'rgb8': (3, cv2.COLOR_RGB2BGR),
    'bgr8': (3, None),
    'rgba8': (4, cv2.COLOR_RGBA2BGR),
    'bgra8': (4, cv2.COLOR_BGRA2BGR),
    'mono8': (1, None),
}


class PanelCamera(Node):

    def __init__(self):
        super().__init__('panel_camera')

        self.declare_parameter('input_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('output_topic', '/athena/camera/compressed')
        self.declare_parameter('rate_hz', 5.0)
        # 0 = keep the camera's width.  320 quarters the pixel count.
        self.declare_parameter('max_width', 0)
        self.declare_parameter('jpeg_quality', 70)

        self.in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.period = 1.0 / max(0.1, float(self.get_parameter('rate_hz').value))
        self.max_w = int(self.get_parameter('max_width').value)
        self.quality = min(100, max(1, int(self.get_parameter('jpeg_quality').value)))

        self.pub = self.create_publisher(CompressedImage, out_topic, OUT_QOS)
        self.sub = None
        self._next = 0.0

        self.create_timer(1.0, self._check_demand)
        self.get_logger().info(
            f'{self.in_topic} -> {out_topic} as JPEG q{self.quality}, '
            f'{1.0 / self.period:.1f} Hz, max width {self.max_w or "native"}; '
            'idle until something subscribes')

    # ------------------------------------------------------------- demand
    def _check_demand(self):
        wanted = self.pub.get_subscription_count() > 0
        if wanted and self.sub is None:
            self.sub = self.create_subscription(
                Image, self.in_topic, self._image_cb, RAW_QOS, raw=True)
            self.get_logger().info(f'viewer connected, reading {self.in_topic}')
        elif not wanted and self.sub is not None:
            self.destroy_subscription(self.sub)
            self.sub = None
            self.get_logger().info(f'no viewer, released {self.in_topic}')

    # ------------------------------------------------------------- frames
    def _image_cb(self, raw):
        now = time.monotonic()
        if now < self._next:
            return                       # throttled: dropped as raw bytes
        # Keep the average at rate_hz, but never schedule into the past (a
        # long gap would otherwise let a burst of frames through).
        self._next = max(self._next + self.period, now)

        msg = deserialize_message(raw, Image)
        img = self._to_bgr(msg)
        if img is None:
            return
        if 0 < self.max_w < img.shape[1]:
            h = max(1, round(img.shape[0] * self.max_w / img.shape[1]))
            img = cv2.resize(img, (self.max_w, h), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            self.get_logger().warn('JPEG encode failed', throttle_duration_sec=30.0)
            return

        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = buf.tobytes()
        self.pub.publish(out)

    def _to_bgr(self, msg):
        spec = ENCODINGS.get(msg.encoding.lower())
        if spec is None:
            self.get_logger().warn(
                f'unsupported encoding {msg.encoding!r}', throttle_duration_sec=30.0)
            return None
        ch, conversion = spec
        h, w = msg.height, msg.width
        rows = np.frombuffer(msg.data, dtype=np.uint8)[:msg.step * h].reshape(h, msg.step)
        img = rows[:, :w * ch]           # drop any row padding
        img = img.reshape(h, w) if ch == 1 else img.reshape(h, w, ch)
        return cv2.cvtColor(img, conversion) if conversion is not None else img


def main(args=None):
    rclpy.init(args=args)
    node = PanelCamera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

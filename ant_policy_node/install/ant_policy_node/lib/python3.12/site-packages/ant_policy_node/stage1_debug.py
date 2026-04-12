#!/usr/bin/env python3
"""Stage 1 vision debug node — saves annotated images showing detection result.

Run alongside a simulation to capture what the camera sees at each grid position
and whether the port detector fires.

Usage (in eval container, after sourcing workspace):
    python3 stage1_debug.py

Saves to /tmp/stage1_debug/<timestamp>_<tag>.png
Each image is a 4-panel composite:
  original RGB | SFP mask (HSV blue/cyan) | SC mask (Otsu bright) | annotated overlay
"""

import os
import time
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image


SAVE_DIR = "/tmp/stage1_debug"
os.makedirs(SAVE_DIR, exist_ok=True)


def _score_contours(contours, min_area, max_area, cx_img, cy_img):
    """Filter and score contours; return sorted list of (score, u, v, area, cnt)."""
    candidates = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area or area > max_area:
            continue
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        u = M["m10"] / M["m00"]
        v = M["m01"] / M["m00"]
        perimeter = cv2.arcLength(cnt, True)
        if perimeter < 1:
            continue
        circularity = 4 * np.pi * area / (perimeter ** 2)
        dist_from_centre = np.hypot(u - cx_img, v - cy_img)
        score = area * circularity / (dist_from_centre + 1.0)
        candidates.append((score, u, v, area, cnt))
    candidates.sort(reverse=True)
    return candidates


def detect_sfp(image_msg, img):
    """HSV blue/cyan detection — mirrors ANT._detect_port_pixel for zone='sfp'."""
    cx_img = image_msg.width / 2.0
    cy_img = image_msg.height / 2.0
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, np.array([80, 80, 80]), np.array([135, 255, 255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = 100
    max_area = image_msg.width * image_msg.height * 0.20
    candidates = _score_contours(contours, min_area, max_area, cx_img, cy_img)
    result = None
    if candidates and candidates[0][0] >= 1.0:
        score, u, v, area, _ = candidates[0]
        result = (u, v, area, score)
    return result, mask, [c[4] for c in candidates]


def detect_sc(image_msg, img):
    """Bright-square detection — mirrors ANT._detect_port_pixel for zone='sc'."""
    cx_img = image_msg.width / 2.0
    cy_img = image_msg.height / 2.0
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    _, mask = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = 500
    max_area = image_msg.width * image_msg.height * 0.50
    candidates = _score_contours(contours, min_area, max_area, cx_img, cy_img)
    result = None
    if candidates and candidates[0][0] >= 1.0:
        score, u, v, area, _ = candidates[0]
        result = (u, v, area, score)
    return result, mask, [c[4] for c in candidates]


class Stage1DebugNode(Node):
    def __init__(self):
        super().__init__("stage1_debug")
        self._frame_count = 0
        self.create_subscription(Image, "/center_camera/image", self._image_cb, 10)
        self.get_logger().info(f"stage1_debug: saving 4-panel frames to {SAVE_DIR}")

    def _image_cb(self, msg):
        self._frame_count += 1

        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3)
        )
        h, w = img.shape[:2]

        sfp_result, sfp_mask, _ = detect_sfp(msg, img)
        sc_result,  sc_mask,  _ = detect_sc(msg, img)

        # ---- Annotated overlay (BGR) ---
        annotated = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.line(annotated, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2), (0, 255, 0), 1)
        cv2.line(annotated, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20), (0, 255, 0), 1)

        tag_parts = []
        if sfp_result is not None:
            u, v, area, score = sfp_result
            cv2.circle(annotated, (int(u), int(v)), 8, (0, 200, 255), 2)   # orange = SFP
            cv2.putText(annotated, f"SFP a={area:.0f} s={score:.1f}",
                        (int(u) + 10, int(v) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)
            tag_parts.append(f"sfp{area:.0f}")

        if sc_result is not None:
            u, v, area, score = sc_result
            cv2.circle(annotated, (int(u), int(v)), 8, (255, 60, 180), 2)   # magenta = SC
            cv2.putText(annotated, f"SC a={area:.0f} s={score:.1f}",
                        (int(u) + 10, int(v) + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 60, 180), 1)
            tag_parts.append(f"sc{area:.0f}")

        if not tag_parts:
            cv2.putText(annotated, "NO DETECTION", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        tag = "_".join(tag_parts) if tag_parts else "none"

        # ---- 4-panel composite: RGB | SFP mask | SC mask | annotated ---
        sfp_bgr = cv2.cvtColor(sfp_mask, cv2.COLOR_GRAY2BGR)
        sc_bgr  = cv2.cvtColor(sc_mask,  cv2.COLOR_GRAY2BGR)
        composite = np.hstack([
            cv2.cvtColor(img, cv2.COLOR_RGB2BGR),
            sfp_bgr,
            sc_bgr,
            annotated,
        ])

        ts = int(time.time() * 1000)
        fname = os.path.join(SAVE_DIR, f"{ts}_{tag}.png")
        cv2.imwrite(fname, composite)


def main():
    rclpy.init()
    node = Stage1DebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

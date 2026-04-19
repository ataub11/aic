"""Port-aperture detectors used by Stage 1 of the ANT policy.

The detector is pluggable so a learned model (YOLO-nano, etc.) can be dropped
in without touching ANT.py.  Each detector takes a `sensor_msgs/Image` and
returns either a pixel `(u, v)` or None.  Classical detectors mirror the HSV
and bright-Otsu logic proven in Runs 15–23; the ONNX path loads an external
model if its weights are present and falls through to classical otherwise.

The current ANT.py Stage 1 uses SFP fast path + SC calibrated-zone fallback
(no grid scan) — that stable path is not disturbed here.  When future work
re-enables vision localisation, instantiate a detector via `make_detector()`
and call `detector.detect(image_msg, context)` in place of the removed
`_detect_port_pixel_sc_prescan`.

Back-projection (pixel → base_link XY) is also provided here so the whole
vision pipeline can be re-enabled from one module once Stage 1 rewiring is
ready.
"""

from __future__ import annotations

import os
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np


Pixel = Tuple[int, int]


@dataclass
class DetectorContext:
    """Optional state passed to a detector — keeps the interface stateless.

    `prior_pixel`: last detection this trial, for temporal proximity filtering.
    `camera_k`:    3x3 intrinsics as a numpy array (for geometry-aware detectors).
    `arm_is_stalled`: True if the arm has not moved for several iterations —
                    detectors can use this to reject arm-fixed features.
    """
    prior_pixel: Optional[Pixel] = None
    camera_k: Optional[np.ndarray] = None
    arm_is_stalled: bool = False
    extras: dict = field(default_factory=dict)


class PortDetector(ABC):
    """Abstract interface for port-aperture detectors."""

    name: str = "base"

    @abstractmethod
    def detect(self, bgr: np.ndarray, context: Optional[DetectorContext] = None) -> Optional[Pixel]:
        """Return pixel (u, v) of the port centre or None on failure."""

    @staticmethod
    def image_msg_to_bgr(image_msg) -> Optional[np.ndarray]:
        """Convert a sensor_msgs/Image to an OpenCV BGR array.

        Handles the common encodings used in the AIC simulator (rgb8, bgr8).
        cv_bridge isn't a required dep of this package so we decode by hand.
        """
        if image_msg is None:
            return None
        enc = getattr(image_msg, "encoding", "bgr8")
        h = int(image_msg.height)
        w = int(image_msg.width)
        buf = np.frombuffer(bytes(image_msg.data), dtype=np.uint8)
        if buf.size != h * w * 3:
            return None
        img = buf.reshape((h, w, 3))
        if enc == "rgb8":
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img.copy()


# ---------------------------------------------------------------------------
# Classical detectors
# ---------------------------------------------------------------------------

class ClassicalSFPDetector(PortDetector):
    """HSV segmentation for the blue/cyan SFP bracket face + Otsu aperture refinement.

    Two-pass detection (Bug 33 fix):
      1. HSV mask H ∈ [80, 135], S/V > 80 → bracket bounding box.
      2. THRESH_BINARY_INV Otsu inside the bbox (+8 px pad) → dark aperture
         centroid at the port face.
    The aperture is at `connector_z` so back-projection is accurate; the
    bracket face alone gives a +4 cm/+10 cm Z-parallax error.

    Known limitation (Bug 37/38): at stall poses 20+ cm from target, the HSV
    can pick up blue/cyan elements on adjacent boards.  Use a ROI mask in
    `context.extras["roi_uv"]` when called from a grid loop.
    """

    name = "classical-sfp"

    def __init__(
        self,
        hsv_lower: Sequence[int] = (80, 80, 80),
        hsv_upper: Sequence[int] = (135, 255, 255),
        min_area_px: int = 400,
        aperture_min_area_px: int = 30,
        roi_pad_px: int = 8,
    ):
        self.hsv_lower = np.asarray(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.asarray(hsv_upper, dtype=np.uint8)
        self.min_area_px = min_area_px
        self.aperture_min_area_px = aperture_min_area_px
        self.roi_pad_px = roi_pad_px

    def detect(self, bgr, context=None):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # Apply optional ROI mask to reject out-of-board detections (Bug 37).
        if context is not None and "roi_uv" in context.extras:
            u0, v0, u1, v1 = context.extras["roi_uv"]
            roi = np.zeros_like(mask)
            roi[v0:v1, u0:u1] = 255
            mask = cv2.bitwise_and(mask, roi)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(cnt) < self.min_area_px:
            return None

        # Aperture refinement: dark-blob Otsu inside the bracket bbox.
        x, y, w, h = cv2.boundingRect(cnt)
        pad = self.roi_pad_px
        h_img, w_img = bgr.shape[:2]
        x0 = max(0, x - pad); y0 = max(0, y - pad)
        x1 = min(w_img, x + w + pad); y1 = min(h_img, y + h + pad)
        roi = cv2.cvtColor(bgr[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
        _, otsu = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        ap_contours, _ = cv2.findContours(otsu, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for ac in sorted(ap_contours, key=cv2.contourArea, reverse=True):
            if cv2.contourArea(ac) < self.aperture_min_area_px:
                break
            M = cv2.moments(ac)
            if M["m00"] > 0:
                u = int(M["m10"] / M["m00"]) + x0
                v = int(M["m01"] / M["m00"]) + y0
                return (u, v)

        # Fallback to bracket centroid — less accurate (parallax error) but better than None.
        M = cv2.moments(cnt)
        if M["m00"] > 0:
            return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        return None


class ClassicalSCDetector(PortDetector):
    """Bright-Otsu detector for the SC port housing (Bug 34 fix).

    max_area=10_000 px² rejects the yellow cable plug (~74 kpx²) that
    otherwise dominates the frame.  An optional centre-mask suppresses the
    arm-fixed plug when the arm is directly above the port.
    """

    name = "classical-sc"

    def __init__(
        self,
        min_area_px: int = 300,
        max_area_px: int = 10_000,
        centre_mask_fraction: float = 0.0,   # 0.0 = disabled; 0.3 masks central 30 % of frame
    ):
        self.min_area_px = min_area_px
        self.max_area_px = max_area_px
        self.centre_mask_fraction = centre_mask_fraction

    def detect(self, bgr, context=None):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        _, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        if self.centre_mask_fraction > 0:
            h, w = otsu.shape
            cy, cx = h // 2, w // 2
            ry, rx = int(h * self.centre_mask_fraction / 2), int(w * self.centre_mask_fraction / 2)
            otsu[cy - ry:cy + ry, cx - rx:cx + rx] = 0

        contours, _ = cv2.findContours(otsu, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_area = -1.0
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area_px or area > self.max_area_px:
                continue
            if area > best_area:
                best_area = area
                best = c
        if best is None:
            return None
        M = cv2.moments(best)
        if M["m00"] <= 0:
            return None
        return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


# ---------------------------------------------------------------------------
# Learned detector (optional ONNX)
# ---------------------------------------------------------------------------

class ONNXPortDetector(PortDetector):
    """Loads an ONNX model for port detection.  Returns None if weights absent.

    Expected model signature:
      input:  NCHW float32 image normalised to [0, 1] at model_input_size
      output: one of
        - (N, 4) bbox xyxy in model pixel coords → centroid used
        - (N, 2) keypoint uv in model pixel coords
      A confidence head (N, 1) at index 1 is optional; threshold filters detections.

    Weights are loaded lazily; if onnxruntime or the file is missing the
    detector transparently returns None so the CompositeDetector can fall
    back to classical.  Calling code should not treat this as an error.
    """

    name = "onnx"

    def __init__(
        self,
        weights_path: str,
        input_size: Tuple[int, int] = (320, 320),
        conf_threshold: float = 0.5,
    ):
        self.weights_path = weights_path
        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self._session = None
        self._load_attempted = False

    def _lazy_load(self) -> bool:
        if self._session is not None:
            return True
        if self._load_attempted:
            return False
        self._load_attempted = True
        if not os.path.exists(self.weights_path):
            return False
        try:
            import onnxruntime as ort   # type: ignore
            self._session = ort.InferenceSession(
                self.weights_path, providers=["CPUExecutionProvider"]
            )
            return True
        except Exception:
            return False

    def detect(self, bgr, context=None):
        if not self._lazy_load():
            return None
        session = self._session
        h_img, w_img = bgr.shape[:2]
        iw, ih = self.input_size
        resized = cv2.resize(bgr, (iw, ih))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        x = rgb.transpose(2, 0, 1)[None, ...]
        try:
            outputs = session.run(None, {session.get_inputs()[0].name: x})
        except Exception:
            return None
        if not outputs:
            return None
        out0 = outputs[0]
        if out0.ndim < 2 or out0.shape[0] == 0:
            return None
        # Accept either (N, 4) bbox or (N, 2) keypoint in model pixel coords.
        det = out0[0]
        if det.shape[-1] >= 4:
            u_model = 0.5 * (det[0] + det[2])
            v_model = 0.5 * (det[1] + det[3])
        elif det.shape[-1] >= 2:
            u_model = det[0]
            v_model = det[1]
        else:
            return None
        # Optional confidence check on outputs[1] if provided.
        if len(outputs) > 1 and outputs[1].size > 0:
            conf = float(outputs[1].flat[0])
            if conf < self.conf_threshold:
                return None
        u = int(u_model * (w_img / iw))
        v = int(v_model * (h_img / ih))
        return (u, v)


# ---------------------------------------------------------------------------
# Composite detector
# ---------------------------------------------------------------------------

class CompositeDetector(PortDetector):
    """Tries each detector in order, returning the first successful detection."""

    name = "composite"

    def __init__(self, detectors: Sequence[PortDetector]):
        self.detectors = list(detectors)

    def detect(self, bgr, context=None):
        for d in self.detectors:
            try:
                px = d.detect(bgr, context)
            except Exception:
                px = None
            if px is not None:
                return px
        return None


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def make_detector(zone: str, onnx_weights_dir: Optional[str] = None) -> PortDetector:
    """Build the default detector for a zone.

    Learned detector is tried first if a weights file exists at
    `{onnx_weights_dir}/{zone}.onnx`; otherwise the classical detector is used
    alone.  `ANT_DETECTOR_WEIGHTS_DIR` env var is honoured when `onnx_weights_dir`
    is not provided.
    """
    if onnx_weights_dir is None:
        onnx_weights_dir = os.environ.get("ANT_DETECTOR_WEIGHTS_DIR")

    classical = ClassicalSFPDetector() if zone == "sfp" else ClassicalSCDetector()

    if onnx_weights_dir:
        candidate = os.path.join(onnx_weights_dir, f"{zone}.onnx")
        if os.path.exists(candidate):
            return CompositeDetector([ONNXPortDetector(candidate), classical])

    return classical


# ---------------------------------------------------------------------------
# Pixel → base_link XY back-projection
# ---------------------------------------------------------------------------

def back_project_to_base_xy(
    u: float, v: float,
    camera_k: np.ndarray,
    cam_in_base: np.ndarray,
    plane_z: float,
) -> Optional[Tuple[float, float]]:
    """Ray from camera through pixel (u, v), intersected with z = plane_z in base_link.

    camera_k:    3x3 intrinsic matrix.
    cam_in_base: 4x4 homogeneous transform from optical frame into base_link.
    plane_z:     Z coordinate of the target plane in base_link.

    Returns (x, y) in base_link, or None if the ray is parallel to the plane.
    """
    if camera_k is None or cam_in_base is None:
        return None
    fx = float(camera_k[0, 0]); fy = float(camera_k[1, 1])
    cx = float(camera_k[0, 2]); cy = float(camera_k[1, 2])
    # Optical frame convention: +Z forward, +X right, +Y down.
    ray_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0, 0.0])
    origin_cam = np.array([0.0, 0.0, 0.0, 1.0])
    ray_base = cam_in_base @ ray_cam
    origin_base = cam_in_base @ origin_cam
    dz = ray_base[2]
    if abs(dz) < 1e-9:
        return None
    t = (plane_z - origin_base[2]) / dz
    if t <= 0:
        return None
    x = origin_base[0] + t * ray_base[0]
    y = origin_base[1] + t * ray_base[1]
    return (float(x), float(y))

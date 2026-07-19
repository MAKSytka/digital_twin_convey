"""Stateful OpenCV detector for a continuous overhead-camera stream.

The contour and ORB stages are adapted from the existing single-image
``BoxDetector.exe`` pipeline.  Stream-specific state is kept in memory:
calibration, an empty-scene reference and loaded dataset descriptors are built
once instead of being recomputed for every frame.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import glob
import math

import cv2
import numpy as np


@dataclass(slots=True)
class DetectorConfig:
    field_length_m: float = 7.70
    field_width_m: float = 0.90
    field_min_area_ratio: float = 0.05
    box_min_area_ratio: float = 0.00015
    box_max_area_ratio: float = 0.50
    aspect_tolerance: float = 0.40
    orb_features: int = 500
    orb_match_ratio: float = 0.75
    min_good_matches: int = 8
    inner_erode_px: int = 6
    use_background_subtraction: bool = True
    background_threshold: int = 25
    minimum_contour_area_px: float = 30.0


@dataclass(slots=True)
class Detection:
    center_local_m: tuple[float, float]
    length_m: float
    width_m: float
    angle_image_rad: float
    box_points_px: np.ndarray
    center_px: tuple[float, float]
    box_type: str
    match_score: int
    confidence: float


def order_corners(points: np.ndarray) -> np.ndarray:
    """Return points as top-left, top-right, bottom-right, bottom-left."""
    pts = np.asarray(points, dtype=np.float32)
    if pts.shape != (4, 2):
        raise ValueError(f"Expected four 2D points, got shape {pts.shape}")
    sums = pts.sum(axis=1)
    differences = np.diff(pts, axis=1).reshape(-1)
    return np.asarray(
        [
            pts[np.argmin(sums)],
            pts[np.argmin(differences)],
            pts[np.argmax(sums)],
            pts[np.argmax(differences)],
        ],
        dtype=np.float32,
    )


def find_coordinate_field(
    image: np.ndarray,
    minimum_area_ratio: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Find the largest field contour and return its corners and filled mask."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 30, 100)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    edges = cv2.dilate(edges, kernel, iterations=2)

    contours, _ = cv2.findContours(
        edges,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if not contours:
        raise RuntimeError("No contours found while calibrating the vision field")

    image_area = float(image.shape[0] * image.shape[1])
    valid = [
        contour
        for contour in contours
        if cv2.contourArea(contour) > image_area * minimum_area_ratio
    ]
    if not valid:
        raise RuntimeError(
            "No calibration contour is large enough; check camera pose and field frame"
        )

    field_contour = max(valid, key=cv2.contourArea)
    perimeter = cv2.arcLength(field_contour, True)
    approximation = cv2.approxPolyDP(field_contour, 0.02 * perimeter, True)
    if len(approximation) == 4:
        corners = order_corners(approximation.reshape(4, 2))
    else:
        corners = order_corners(cv2.boxPoints(cv2.minAreaRect(field_contour)))

    field_mask = np.zeros(gray.shape, dtype=np.uint8)
    cv2.fillConvexPoly(field_mask, corners.astype(np.int32), 255)
    return corners, field_mask


def build_homography(
    field_corners: np.ndarray,
    field_length_m: float,
    field_width_m: float,
) -> np.ndarray:
    destination = np.asarray(
        [
            [0.0, 0.0],
            [field_length_m, 0.0],
            [field_length_m, field_width_m],
            [0.0, field_width_m],
        ],
        dtype=np.float32,
    )
    homography, _ = cv2.findHomography(field_corners, destination)
    if homography is None:
        raise RuntimeError("Could not build camera-to-field homography")
    return homography


def points_to_field(homography: np.ndarray, points_px: np.ndarray) -> np.ndarray:
    points = np.asarray(points_px, dtype=np.float32).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(points, homography).reshape(-1, 2)


class StreamingBoxDetector:
    """Calibrate once and process a sequence of camera frames."""

    def __init__(self, config: DetectorConfig, dataset_folder: str = "") -> None:
        self.config = config
        self.field_corners: np.ndarray | None = None
        self.field_mask: np.ndarray | None = None
        self.homography: np.ndarray | None = None
        self.background_bgr: np.ndarray | None = None
        self.background_gray: np.ndarray | None = None
        self.orb = cv2.ORB_create(config.orb_features)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.templates = self._load_templates(dataset_folder)

    @property
    def is_calibrated(self) -> bool:
        return self.homography is not None and self.field_mask is not None

    def calibrate(self, empty_scene_bgr: np.ndarray) -> np.ndarray:
        """Calibrate the field and freeze an empty-scene reference image."""
        corners, mask = find_coordinate_field(
            empty_scene_bgr,
            self.config.field_min_area_ratio,
        )
        self.field_corners = corners
        self.field_mask = mask
        self.homography = build_homography(
            corners,
            self.config.field_length_m,
            self.config.field_width_m,
        )
        self.set_background(empty_scene_bgr)
        return corners

    def set_background(self, empty_scene_bgr: np.ndarray) -> None:
        self.background_bgr = empty_scene_bgr.copy()
        self.background_gray = cv2.cvtColor(
            self.background_bgr,
            cv2.COLOR_BGR2GRAY,
        )

    def detect(self, image: np.ndarray) -> tuple[list[Detection], np.ndarray]:
        if not self.is_calibrated:
            raise RuntimeError("Detector must be calibrated before processing frames")
        assert self.field_mask is not None
        assert self.field_corners is not None
        assert self.homography is not None

        candidates = self._candidate_mask(image)
        contours, _ = cv2.findContours(
            candidates,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        field_area = abs(cv2.contourArea(self.field_corners.astype(np.int32)))
        detections: list[Detection] = []

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.config.minimum_contour_area_px or field_area <= 0.0:
                continue
            ratio_to_field = area / field_area
            if not (
                self.config.box_min_area_ratio
                <= ratio_to_field
                <= self.config.box_max_area_ratio
            ):
                continue

            rect = cv2.minAreaRect(contour)
            (center_x, center_y), (width_px, height_px), _ = rect
            if width_px <= 0.0 or height_px <= 0.0:
                continue

            aspect_ratio = max(width_px, height_px) / min(width_px, height_px)
            if not self._aspect_ratio_is_plausible(aspect_ratio):
                continue

            box_points = cv2.boxPoints(rect)
            field_points = points_to_field(self.homography, box_points)
            center, length_m, width_m, angle_image = self._geometry(field_points)

            # Reject objects outside the product envelope after perspective mapping.
            if not (0.02 <= length_m <= 0.65 and 0.01 <= width_m <= 0.55):
                continue

            rectangle_area = max(width_px * height_px, 1.0)
            rectangularity = min(1.0, max(0.0, area / rectangle_area))
            box_type, match_score = self._classify(image, box_points)

            detections.append(
                Detection(
                    center_local_m=center,
                    length_m=length_m,
                    width_m=width_m,
                    angle_image_rad=angle_image,
                    box_points_px=box_points,
                    center_px=(float(center_x), float(center_y)),
                    box_type=box_type,
                    match_score=match_score,
                    confidence=rectangularity,
                )
            )

        detections.sort(key=lambda item: item.center_local_m[0])
        return detections, candidates

    def _candidate_mask(self, image: np.ndarray) -> np.ndarray:
        assert self.field_mask is not None
        inner_kernel_size = max(1, int(self.config.inner_erode_px))
        inner_mask = cv2.erode(
            self.field_mask,
            np.ones((inner_kernel_size, inner_kernel_size), np.uint8),
        )

        if self.config.use_background_subtraction:
            if self.background_gray is None:
                raise RuntimeError("Background subtraction requested without a reference")
            if self.background_bgr is None:
                raise RuntimeError("Background image is not available")
            color_difference = cv2.absdiff(image, self.background_bgr)
            difference = np.max(color_difference, axis=2).astype(np.uint8)
            difference = cv2.GaussianBlur(difference, (5, 5), 0)
            _, candidates = cv2.threshold(
                difference,
                int(self.config.background_threshold),
                255,
                cv2.THRESH_BINARY,
            )
        else:
            # Original BoxDetector segmentation for real-camera images.
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            colored = cv2.inRange(hsv, (0, 30, 0), (180, 255, 255))
            white = cv2.inRange(hsv, (0, 0, 200), (180, 40, 255))
            candidates = cv2.bitwise_or(colored, white)

        candidates = cv2.bitwise_and(candidates, inner_mask)
        candidates = cv2.morphologyEx(
            candidates,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
        )
        candidates = cv2.morphologyEx(
            candidates,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)),
        )
        candidates = cv2.dilate(
            candidates,
            cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
            iterations=1,
        )
        return candidates

    def draw_debug(
        self,
        image: np.ndarray,
        detections: list[Detection],
        track_ids: list[int] | None = None,
    ) -> np.ndarray:
        result = image.copy()
        if self.field_corners is not None:
            cv2.polylines(
                result,
                [self.field_corners.astype(np.int32)],
                True,
                (0, 255, 255),
                2,
            )
        for index, detection in enumerate(detections):
            points = detection.box_points_px.astype(np.int32)
            cv2.polylines(result, [points], True, (0, 255, 0), 2)
            track_id = track_ids[index] if track_ids is not None else index + 1
            label = (
                f"id={track_id} "
                f"x={detection.center_local_m[0]:.2f} "
                f"y={detection.center_local_m[1]:.2f}"
            )
            x = int(detection.center_px[0])
            y = int(detection.center_px[1])
            cv2.circle(result, (x, y), 3, (0, 0, 255), -1)
            cv2.putText(
                result,
                label,
                (x + 5, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 80, 0),
                1,
                cv2.LINE_AA,
            )
        return result

    def _load_templates(self, dataset_folder: str) -> list[dict[str, object]]:
        if not dataset_folder:
            return []
        folder = Path(dataset_folder).expanduser()
        if not folder.is_dir():
            return []

        files: list[str] = []
        for extension in ("*.jpg", "*.jpeg", "*.png", "*.bmp"):
            files.extend(glob.glob(str(folder / extension)))
            files.extend(glob.glob(str(folder / "**" / extension), recursive=True))

        templates: list[dict[str, object]] = []
        for filename in sorted(set(files)):
            image = cv2.imread(filename)
            if image is None:
                continue
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            _, threshold = cv2.threshold(
                gray,
                0,
                255,
                cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU,
            )
            contours, _ = cv2.findContours(
                threshold,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            aspect_ratio = None
            if contours:
                _, (width, height), _ = cv2.minAreaRect(
                    max(contours, key=cv2.contourArea)
                )
                if min(width, height) > 0.0:
                    aspect_ratio = max(width, height) / min(width, height)
            keypoints, descriptors = self.orb.detectAndCompute(gray, None)
            templates.append(
                {
                    "name": Path(filename).stem,
                    "path": filename,
                    "aspect_ratio": aspect_ratio,
                    "keypoints": keypoints,
                    "descriptors": descriptors,
                }
            )
        return templates

    def _aspect_ratio_is_plausible(self, ratio: float) -> bool:
        known = [
            float(template["aspect_ratio"])
            for template in self.templates
            if template.get("aspect_ratio") is not None
        ]
        if not known:
            return 1.0 <= ratio <= 12.0
        tolerance = self.config.aspect_tolerance
        return any(abs(ratio - expected) / expected <= tolerance for expected in known)

    def _warp_box(self, image: np.ndarray, box_points: np.ndarray) -> np.ndarray:
        points = order_corners(box_points)
        width = int(
            max(
                np.linalg.norm(points[0] - points[1]),
                np.linalg.norm(points[2] - points[3]),
            )
        )
        height = int(
            max(
                np.linalg.norm(points[0] - points[3]),
                np.linalg.norm(points[1] - points[2]),
            )
        )
        width = max(20, min(600, width or 1))
        height = max(20, min(600, height or 1))
        destination = np.asarray(
            [[0, 0], [width, 0], [width, height], [0, height]],
            dtype=np.float32,
        )
        transform = cv2.getPerspectiveTransform(points, destination)
        return cv2.warpPerspective(image, transform, (width, height))

    def _classify(
        self,
        image: np.ndarray,
        box_points: np.ndarray,
    ) -> tuple[str, int]:
        if not self.templates:
            return "unknown", 0
        crop = self._warp_box(image, box_points)
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        _, descriptors = self.orb.detectAndCompute(gray, None)
        if descriptors is None or len(descriptors) < 2:
            return "unknown", 0

        best_name = "unknown"
        best_good = 0
        for template in self.templates:
            template_descriptors = template.get("descriptors")
            if template_descriptors is None or len(template_descriptors) < 2:
                continue
            matches = self.matcher.knnMatch(
                descriptors,
                template_descriptors,
                k=2,
            )
            good = [
                first
                for pair in matches
                if len(pair) == 2
                for first, second in [pair]
                if first.distance < self.config.orb_match_ratio * second.distance
            ]
            if len(good) > best_good:
                best_good = len(good)
                best_name = str(template["name"])
        if best_good < self.config.min_good_matches:
            return "unknown", best_good
        return best_name, best_good

    @staticmethod
    def _geometry(
        field_points: np.ndarray,
    ) -> tuple[tuple[float, float], float, float, float]:
        points = order_corners(field_points)
        first_edge = points[1] - points[0]
        second_edge = points[2] - points[1]
        first_length = float(np.linalg.norm(first_edge))
        second_length = float(np.linalg.norm(second_edge))
        if first_length >= second_length:
            long_edge = first_edge
            length_m = first_length
            width_m = second_length
        else:
            long_edge = second_edge
            length_m = second_length
            width_m = first_length
        angle = math.atan2(float(long_edge[1]), float(long_edge[0]))
        angle = ((angle + math.pi / 2.0) % math.pi) - math.pi / 2.0
        center = points.mean(axis=0)
        return (
            (float(center[0]), float(center[1])),
            length_m,
            width_m,
            angle,
        )

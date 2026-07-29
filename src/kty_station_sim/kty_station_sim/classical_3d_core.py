"""Classical RGB-D carton instance segmentation and tracking core."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import cv2
import numpy as np


@dataclass(slots=True)
class Instance:
    mask: np.ndarray
    contour_px: np.ndarray
    polygon_xy: list[tuple[float, float]]
    rectangle_xy: list[tuple[float, float]]
    centroid: tuple[float, float, float]
    normal: tuple[float, float, float]
    size_xyz: tuple[float, float, float]
    yaw: float
    area_m2: float
    confidence: float
    occlusion: float
    extent: tuple[float, float, float, float]
    track_id: int = 0
    state: str = "VISIBLE"
    top_accessible: bool = False
    clearances: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    grasps: list[dict] = field(default_factory=list)


@dataclass(slots=True)
class Track:
    track_id: int
    centroid: tuple[float, float, float]
    yaw: float
    polygon_xy: list[tuple[float, float]]
    rectangle_xy: list[tuple[float, float]]
    normal: tuple[float, float, float]
    size_xyz: tuple[float, float, float]
    area_m2: float
    confidence: float
    extent: tuple[float, float, float, float]
    misses: int = 0


def wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def unit(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    return vector / norm if norm > 1.0e-9 else np.array((0.0, 0.0, 1.0))


class Tracker:
    def __init__(self, max_distance: float, max_height: float, max_misses: int):
        self.max_distance = max_distance
        self.max_height = max_height
        self.max_misses = max_misses
        self.next_id = 1
        self.tracks: dict[int, Track] = {}

    def update(self, visible: list[Instance]) -> list[Instance]:
        matches: list[tuple[float, int, int]] = []
        for track_id, track in self.tracks.items():
            for index, item in enumerate(visible):
                distance = math.dist(track.centroid[:2], item.centroid[:2])
                height = abs(track.centroid[2] - item.centroid[2])
                size = math.dist(track.size_xyz[:2], item.size_xyz[:2])
                if distance <= self.max_distance and height <= self.max_height and size <= 0.22:
                    matches.append((distance + 0.35 * height + 0.20 * size, track_id, index))

        assigned_tracks: set[int] = set()
        assigned_items: set[int] = set()
        for _, track_id, index in sorted(matches):
            if track_id in assigned_tracks or index in assigned_items:
                continue
            item = visible[index]
            item.track_id = track_id
            track = self.tracks[track_id]
            alpha = 0.65
            track.centroid = tuple(alpha * a + (1.0 - alpha) * b for a, b in zip(item.centroid, track.centroid))
            track.yaw = wrap_angle(track.yaw + alpha * wrap_angle(item.yaw - track.yaw))
            track.polygon_xy = list(item.polygon_xy)
            track.rectangle_xy = list(item.rectangle_xy)
            track.normal = item.normal
            track.size_xyz = item.size_xyz
            track.area_m2 = item.area_m2
            track.confidence = item.confidence
            track.extent = item.extent
            track.misses = 0
            assigned_tracks.add(track_id)
            assigned_items.add(index)

        for index, item in enumerate(visible):
            if index in assigned_items:
                continue
            item.track_id = self.next_id
            self.tracks[self.next_id] = Track(
                track_id=self.next_id,
                centroid=item.centroid,
                yaw=item.yaw,
                polygon_xy=list(item.polygon_xy),
                rectangle_xy=list(item.rectangle_xy),
                normal=item.normal,
                size_xyz=item.size_xyz,
                area_m2=item.area_m2,
                confidence=item.confidence,
                extent=item.extent,
            )
            assigned_tracks.add(self.next_id)
            self.next_id += 1

        occluded: list[Instance] = []
        for track_id, track in list(self.tracks.items()):
            if track_id in assigned_tracks:
                continue
            track.misses += 1
            if track.misses > self.max_misses:
                del self.tracks[track_id]
                continue
            score = min(1.0, 0.45 + 0.55 * track.misses / self.max_misses)
            occluded.append(
                Instance(
                    mask=np.empty((0, 0), np.uint8),
                    contour_px=np.empty((0, 1, 2), np.int32),
                    polygon_xy=list(track.polygon_xy),
                    rectangle_xy=list(track.rectangle_xy),
                    centroid=track.centroid,
                    normal=track.normal,
                    size_xyz=track.size_xyz,
                    yaw=track.yaw,
                    area_m2=track.area_m2,
                    confidence=max(0.05, track.confidence * (1.0 - 0.55 * score)),
                    occlusion=score,
                    extent=track.extent,
                    track_id=track_id,
                    state="OCCLUDED",
                )
            )
        return occluded


class Classical3DSegmenter:
    def __init__(
        self,
        *,
        camera_to_bottom: float,
        internal_length: float,
        internal_width: float,
        internal_height: float,
        minimum_height: float,
        minimum_area_px: float,
        depth_edge_threshold: float,
        normal_edge_threshold: float,
        seed_distance_px: int,
        seed_height_prominence: float,
        top_normal_min_z: float,
        top_occlusion_max: float,
    ):
        self.camera_to_bottom = camera_to_bottom
        self.internal_length = internal_length
        self.internal_width = internal_width
        self.internal_height = internal_height
        self.minimum_height = minimum_height
        self.minimum_area = minimum_area_px
        self.depth_edge_threshold = depth_edge_threshold
        self.normal_edge_threshold = normal_edge_threshold
        self.seed_distance = seed_distance_px
        self.seed_height_prominence = seed_height_prominence
        self.top_normal_min_z = top_normal_min_z
        self.top_occlusion_max = top_occlusion_max

    def segment(self, rgb, depth, intrinsics):
        fx, fy, cx, cy = intrinsics
        rows, cols = depth.shape
        half_u = self.internal_length * fx / (2.0 * self.camera_to_bottom)
        half_v = self.internal_width * fy / (2.0 * self.camera_to_bottom)
        u0, u1 = max(0, int(cx - half_u)), min(cols, int(cx + half_u))
        v0, v1 = max(0, int(cy - half_v)), min(rows, int(cy + half_v))
        roi_mask = np.zeros(depth.shape, np.uint8)
        roi_mask[v0:v1, u0:u1] = 255
        roi_mask = cv2.erode(roi_mask, np.ones((9, 9), np.uint8))
        roi = roi_mask > 0
        finite = np.isfinite(depth) & (depth > 0.15) & (depth < 3.0)
        valid = finite & roi
        valid_fraction = float(np.count_nonzero(valid) / max(1, np.count_nonzero(roi)))

        repaired = self._repair_depth(depth, valid)
        filtered = cv2.bilateralFilter(repaired, 5, 0.012, 4.0)
        height = np.zeros_like(filtered, np.float32)
        height[valid] = np.clip(self.camera_to_bottom - filtered[valid], 0.0, self.internal_height + 0.08)
        mask = (valid & (height >= self.minimum_height) & (height <= self.internal_height + 0.08)).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        normals = self._normals(height, fx, fy)
        boundary = self._boundaries(rgb, filtered, height, normals, mask)
        regions = self._watershed(mask, height, boundary)
        instances = []
        for region in regions:
            item = self._describe(region, filtered, height, normals, boundary, intrinsics)
            if item is not None:
                instances.append(item)
        instances.sort(key=lambda item: (item.centroid[0], item.centroid[1]))
        return instances, roi_mask, height, boundary, valid_fraction

    @staticmethod
    def _repair_depth(depth, valid):
        output = depth.astype(np.float32, copy=True)
        fallback = float(np.nanmedian(output[valid])) if np.any(valid) else 1.25
        output[~np.isfinite(output)] = fallback
        invalid = (~valid).astype(np.uint8)
        normalized = np.clip(output / 3.0 * 255.0, 0, 255).astype(np.uint8)
        repaired = cv2.inpaint(normalized, invalid, 3, cv2.INPAINT_NS)
        output[~valid] = repaired[~valid].astype(np.float32) / 255.0 * 3.0
        return output

    @staticmethod
    def _normals(height, fx, fy):
        dx = cv2.Sobel(height, cv2.CV_32F, 1, 0, ksize=3) / 8.0 / max(1.25 / fx, 1e-6)
        dy = -cv2.Sobel(height, cv2.CV_32F, 0, 1, ksize=3) / 8.0 / max(1.25 / fy, 1e-6)
        nx, ny, nz = -dx, -dy, np.ones_like(height)
        norm = np.maximum(np.sqrt(nx * nx + ny * ny + nz * nz), 1e-6)
        return nx / norm, ny / norm, nz / norm

    def _boundaries(self, rgb, depth, height, normals, mask):
        def gradient(image):
            gx = cv2.Sobel(image, cv2.CV_32F, 1, 0, ksize=3) / 8.0
            gy = cv2.Sobel(image, cv2.CV_32F, 0, 1, ksize=3) / 8.0
            return np.hypot(gx, gy)

        depth_edge = gradient(depth) / max(self.depth_edge_threshold, 1e-5)
        height_edge = gradient(height) / max(self.depth_edge_threshold, 1e-5)
        normal_edge = np.sqrt(sum(gradient(item) ** 2 for item in normals)) / max(self.normal_edge_threshold, 1e-5)
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0
        color_edge = gradient(gray) / 0.10
        result = np.clip(0.42 * depth_edge + 0.28 * height_edge + 0.22 * normal_edge + 0.08 * color_edge, 0.0, 1.0)
        result[mask == 0] = 1.0
        return cv2.GaussianBlur(result, (3, 3), 0.6)

    def _watershed(self, mask, height, boundary):
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        output = []
        for component_id in range(1, count):
            if stats[component_id, cv2.CC_STAT_AREA] < self.minimum_area:
                continue
            component = (labels == component_id).astype(np.uint8) * 255
            seeds = self._seeds(component, height, boundary)
            seed_count, seed_labels = cv2.connectedComponents(seeds)
            if seed_count <= 2:
                output.append(component)
                continue
            markers = np.zeros(mask.shape, np.int32)
            markers[component == 0] = 1
            for seed_id in range(1, seed_count):
                markers[seed_labels == seed_id] = seed_id + 1
            guide = cv2.cvtColor(np.uint8(boundary * 255.0), cv2.COLOR_GRAY2BGR)
            cv2.watershed(guide, markers)
            split = []
            for marker_id in range(2, seed_count + 1):
                region = ((markers == marker_id) & (component > 0)).astype(np.uint8) * 255
                if np.count_nonzero(region) >= self.minimum_area:
                    split.append(region)
            output.extend(split if len(split) > 1 else [component])
        return output

    def _seeds(self, component, height, boundary):
        distance = cv2.distanceTransform(component, cv2.DIST_L2, 5)
        core = cv2.bitwise_and(component, (boundary < 0.52).astype(np.uint8) * 255)
        core = cv2.erode(core, np.ones((3, 3), np.uint8))
        kernel_size = max(7, 2 * (self.seed_distance // 2) + 1)
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        distance_max = cv2.dilate(distance, kernel)
        smoothed_height = cv2.GaussianBlur(height, (7, 7), 1.2)
        height_max = cv2.dilate(smoothed_height, kernel)
        median = float(np.median(smoothed_height[component > 0]))
        peaks = (
            (core > 0)
            & (distance >= np.maximum(3.0, 0.92 * distance_max))
            & ((smoothed_height >= height_max - self.seed_height_prominence) | (smoothed_height >= median + self.seed_height_prominence))
        ).astype(np.uint8) * 255
        count, _, stats, centroids = cv2.connectedComponentsWithStats(peaks)
        candidates = []
        for index in range(1, count):
            if stats[index, cv2.CC_STAT_AREA] < 2:
                continue
            u, v = int(centroids[index, 0]), int(centroids[index, 1])
            candidates.append((float(distance[v, u] + 18.0 * smoothed_height[v, u]), u, v))
        candidates.sort(reverse=True)
        selected = []
        for _, u, v in candidates:
            if all(math.hypot(u - su, v - sv) >= self.seed_distance for su, sv in selected):
                selected.append((u, v))
        if not selected:
            _, _, _, point = cv2.minMaxLoc(distance)
            selected = [(int(point[0]), int(point[1]))]
        seeds = np.zeros_like(component)
        for u, v in selected:
            cv2.circle(seeds, (u, v), max(3, self.seed_distance // 5), 255, -1)
        return cv2.bitwise_and(seeds, component)

    def _describe(self, region, depth, height, normals, boundary, intrinsics):
        fx, fy, cx, cy = intrinsics
        contours, _ = cv2.findContours(region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        area_px = float(cv2.contourArea(contour))
        if area_px < self.minimum_area:
            return None
        vv, uu = np.nonzero(region > 0)
        valid = np.isfinite(depth[vv, uu])
        vv, uu = vv[valid], uu[valid]
        if uu.size == 0:
            return None
        dd = depth[vv, uu]
        xx = (uu - cx) * dd / fx
        yy = -(vv - cy) * dd / fy
        zz = height[vv, uu]
        xy = np.column_stack((xx, yy)).astype(np.float32)
        rectangle = cv2.minAreaRect(xy)
        rectangle_xy = [(float(x), float(y)) for x, y in cv2.boxPoints(rectangle)]
        small, large = sorted((float(rectangle[1][0]), float(rectangle[1][1])))
        yaw = math.radians(rectangle[2] + (90.0 if rectangle[1][0] > rectangle[1][1] else 0.0))
        yaw = wrap_angle(yaw)
        centroid = (float(np.median(xx)), float(np.median(yy)), float(np.percentile(zz, 95.0)))
        median_depth = float(np.median(dd))
        polygon_px = cv2.approxPolyDP(contour, 0.012 * cv2.arcLength(contour, True), True).reshape(-1, 2)
        polygon_xy = [((float(u) - cx) * median_depth / fx, -(float(v) - cy) * median_depth / fy) for u, v in polygon_px]
        if len(polygon_xy) < 3:
            return None
        normal = unit(np.array([np.median(item[vv, uu]) for item in normals], np.float64))
        area_m2 = area_px * median_depth**2 / max(fx * fy, 1e-9)
        rect_area = max(small * large, 1e-6)
        rectangularity = min(1.0, area_m2 / rect_area)
        edge = cv2.dilate(region, np.ones((3, 3), np.uint8)) - region
        ambiguity = float(np.mean(boundary[edge > 0])) if np.any(edge) else 0.0
        occlusion = min(1.0, 0.52 * (1.0 - rectangularity) + 0.30 * ambiguity + 0.18 * max(0.0, 1.0 - normal[2]))
        confidence = float(np.clip(0.50 + 0.30 * rectangularity + 0.20 * (1.0 - ambiguity), 0.05, 1.0))
        xs, ys = zip(*polygon_xy)
        return Instance(
            mask=region,
            contour_px=contour,
            polygon_xy=polygon_xy,
            rectangle_xy=rectangle_xy,
            centroid=centroid,
            normal=(float(normal[0]), float(normal[1]), float(normal[2])),
            size_xyz=(large, small, centroid[2]),
            yaw=yaw,
            area_m2=float(area_m2),
            confidence=confidence,
            occlusion=occlusion,
            extent=(min(xs), max(xs), min(ys), max(ys)),
            top_accessible=normal[2] >= self.top_normal_min_z and occlusion <= self.top_occlusion_max and confidence >= 0.58,
        )

"""Pure control helpers for the V7 immutable global queue.

The functions in this module do not depend on ROS 2.  They are intentionally
kept small so the speed-profile logic can be unit-tested without launching
Gazebo.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True, slots=True)
class GapState:
    """Longitudinal state of one product in immutable queue order."""

    uid: int
    x: float
    half_length: float
    vx: float
    target_gap_to_follower: float


@dataclass(frozen=True, slots=True)
class PairwiseProfile:
    """Result of the direct adjacent-gap controller."""

    speed_by_uid: dict[int, float]
    urgency_by_uid: dict[int, float]
    clearance_by_pair: tuple[float, ...]
    required_delta_by_pair: tuple[float, ...]
    inversion_count: int


def _fit_offsets_to_limits(
    raw_offsets: Sequence[float],
    transport_speed: float,
    minimum_speed: float,
    maximum_speed: float,
) -> list[float]:
    """Fit an ordered relative-speed profile into actuator limits.

    ``raw_offsets`` is expected to be non-increasing from queue leader to tail.
    Small corrections retain their requested magnitude.  If the requested span
    exceeds the available 1..3 m/s window, it is scaled uniformly, preserving
    all relative priorities rather than clipping several products to the same
    limit.
    """

    if not raw_offsets:
        return []

    raw_min = min(raw_offsets)
    raw_max = max(raw_offsets)
    raw_span = raw_max - raw_min
    available_span = maximum_speed - minimum_speed
    scale = 1.0
    if raw_span > available_span and raw_span > 1.0e-9:
        scale = available_span / raw_span

    mean_offset = sum(raw_offsets) / len(raw_offsets)
    values = [
        transport_speed + scale * (offset - mean_offset)
        for offset in raw_offsets
    ]

    # A common shift keeps all pairwise differences unchanged.
    high_excess = max(values) - maximum_speed
    if high_excess > 0.0:
        values = [value - high_excess for value in values]

    low_deficit = minimum_speed - min(values)
    if low_deficit > 0.0:
        values = [value + low_deficit for value in values]

    # Numerical guard.  With the span scaling above, clipping should be tiny.
    return [
        clamp(value, minimum_speed, maximum_speed)
        for value in values
    ]


def build_pairwise_speed_profile(
    ordered: Sequence[GapState],
    *,
    transport_speed: float,
    minimum_speed: float,
    maximum_speed: float,
    gap_gain: float,
    relative_velocity_gain: float,
    maximum_relative_speed: float,
    inversion_margin: float,
) -> PairwiseProfile:
    """Create product speeds from adjacent queue gaps.

    The queue order is immutable.  For every adjacent pair the controller asks
    for a positive leader-minus-follower speed difference while the measured
    clearance is below its target.  The pair requests are accumulated and then
    fitted into the available speed range.
    """

    if not ordered:
        return PairwiseProfile({}, {}, (), (), 0)
    if len(ordered) == 1:
        item = ordered[0]
        return PairwiseProfile(
            {item.uid: clamp(transport_speed, minimum_speed, maximum_speed)},
            {item.uid: 0.0},
            (),
            (),
            0,
        )

    clearances: list[float] = []
    pair_deltas: list[float] = []
    pair_urgencies: list[float] = []
    inversions = 0

    for leader, follower in zip(ordered, ordered[1:]):
        clearance = (
            leader.x
            - leader.half_length
            - follower.x
            - follower.half_length
        )
        clearances.append(clearance)

        target_gap = max(0.0, leader.target_gap_to_follower)
        deficit = max(0.0, target_gap - clearance)
        closing_speed = max(0.0, follower.vx - leader.vx)
        requested_delta = (
            gap_gain * deficit
            + relative_velocity_gain * closing_speed
        )

        # The follower has physically moved in front of its immutable leader.
        # Use the full differential range until the inversion is removed.
        if follower.x > leader.x + inversion_margin:
            inversions += 1
            requested_delta = maximum_relative_speed

        requested_delta = clamp(
            requested_delta,
            0.0,
            maximum_relative_speed,
        )
        pair_deltas.append(requested_delta)
        pair_urgencies.append(
            deficit / max(target_gap, 0.03)
            + 0.25 * closing_speed
        )

    # Tail offset is zero.  Every preceding product receives the sum of all
    # required leader-minus-follower differences behind it.
    raw_offsets = [0.0] * len(ordered)
    running = 0.0
    for index in range(len(pair_deltas) - 1, -1, -1):
        running += pair_deltas[index]
        raw_offsets[index] = running

    fitted = _fit_offsets_to_limits(
        raw_offsets,
        transport_speed,
        minimum_speed,
        maximum_speed,
    )

    urgency_by_uid: dict[int, float] = {}
    for index, item in enumerate(ordered):
        adjacent: list[float] = []
        if index > 0:
            adjacent.append(pair_urgencies[index - 1])
        if index < len(pair_urgencies):
            adjacent.append(pair_urgencies[index])
        urgency_by_uid[item.uid] = max(adjacent, default=0.0)

    return PairwiseProfile(
        speed_by_uid={
            item.uid: fitted[index]
            for index, item in enumerate(ordered)
        },
        urgency_by_uid=urgency_by_uid,
        clearance_by_pair=tuple(clearances),
        required_delta_by_pair=tuple(pair_deltas),
        inversion_count=inversions,
    )


def cosine_similarity(first: Sequence[float], second: Sequence[float]) -> float:
    """Return cosine similarity for two non-negative contact vectors."""

    if len(first) != len(second):
        raise ValueError("contact vectors must have equal length")
    numerator = sum(a * b for a, b in zip(first, second))
    first_norm = math.sqrt(sum(value * value for value in first))
    second_norm = math.sqrt(sum(value * value for value in second))
    if first_norm <= 1.0e-12 or second_norm <= 1.0e-12:
        return 0.0
    return numerator / (first_norm * second_norm)

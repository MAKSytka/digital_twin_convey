#!/usr/bin/env python3
"""Deterministic tests for the V7 pure pairwise-gap controller."""

from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "singulator_control"))

from singulator_control.global_queue_logic import (  # noqa: E402
    GapState,
    allocate_cell_speeds,
    build_pairwise_speed_profile,
)


def build(xs: list[float], target_gap: float = 0.18):
    states = [
        GapState(
            uid=index + 1,
            x=x,
            half_length=0.10,
            vx=2.0,
            target_gap_to_follower=target_gap,
        )
        for index, x in enumerate(xs)
    ]
    return build_pairwise_speed_profile(
        states,
        transport_speed=2.50,
        minimum_speed=1.00,
        maximum_speed=3.00,
        gap_gain=3.00,
        relative_velocity_gain=0.50,
        maximum_relative_speed=2.00,
        inversion_margin=0.03,
    )


def deadline_profile():
    states = [
        GapState(1, 4.90, 0.10, 2.5, 0.18),
        GapState(2, 4.90, 0.10, 2.5, 0.18),
    ]
    return build_pairwise_speed_profile(
        states,
        transport_speed=2.50,
        minimum_speed=1.00,
        maximum_speed=3.00,
        gap_gain=3.00,
        relative_velocity_gain=0.50,
        maximum_relative_speed=2.00,
        inversion_margin=0.03,
        minimum_delta_by_pair=[1.50],
    )


def assert_bounded(speeds: dict[int, float]) -> None:
    assert speeds
    assert min(speeds.values()) >= 1.00 - 1.0e-9
    assert max(speeds.values()) <= 3.00 + 1.0e-9


def effective_speed(
    speeds: list[float], contacts: list[tuple[int, float]]
) -> float:
    total = sum(overlap for _, overlap in contacts)
    return sum(speeds[index] * overlap for index, overlap in contacts) / total



def simulate_transverse_wave() -> tuple[list[float], list[float]]:
    xs = [0.0, 0.0, 0.0, 0.0]
    velocities = [2.0, 2.0, 2.0, 2.0]
    dt = 0.02
    for _ in range(150):
        states = [
            GapState(
                uid=index + 1,
                x=xs[index],
                half_length=0.10,
                vx=velocities[index],
                target_gap_to_follower=0.18,
            )
            for index in range(4)
        ]
        profile = build_pairwise_speed_profile(
            states,
            transport_speed=2.50,
            minimum_speed=1.00,
            maximum_speed=3.00,
            gap_gain=3.00,
            relative_velocity_gain=0.50,
            maximum_relative_speed=2.00,
            inversion_margin=0.03,
        )
        velocities = [profile.speed_by_uid[index] for index in range(1, 5)]
        xs = [x + velocity * dt for x, velocity in zip(xs, velocities)]
    clearances = [
        xs[index] - 0.10 - xs[index + 1] - 0.10
        for index in range(3)
    ]
    return xs, clearances

def main() -> None:
    transverse = build([0.0, 0.0, 0.0, 0.0])
    transverse_speeds = [transverse.speed_by_uid[index] for index in range(1, 5)]
    assert_bounded(transverse.speed_by_uid)
    assert all(
        first > second
        for first, second in zip(transverse_speeds, transverse_speeds[1:])
    ), transverse_speeds
    assert transverse_speeds[0] == 3.0
    assert transverse_speeds[-1] == 1.0

    separated = build([1.20, 0.80, 0.40, 0.0])
    assert_bounded(separated.speed_by_uid)
    assert all(
        abs(speed - 2.50) < 1.0e-9
        for speed in separated.speed_by_uid.values()
    ), separated.speed_by_uid

    partially_separated = build([0.70, 0.40, 0.20, 0.0])
    partial = [
        partially_separated.speed_by_uid[index]
        for index in range(1, 5)
    ]
    assert_bounded(partially_separated.speed_by_uid)
    assert partial[0] >= partial[1] >= partial[2] >= partial[3]

    inverted = build([0.0, 0.10])
    assert inverted.inversion_count == 1
    assert inverted.speed_by_uid[1] > inverted.speed_by_uid[2]

    deadline = deadline_profile()
    assert deadline.required_delta_by_pair == (1.50,)
    assert deadline.speed_by_uid[1] - deadline.speed_by_uid[2] >= 1.49

    _, simulated_clearances = simulate_transverse_wave()
    assert min(simulated_clearances) >= 0.17, simulated_clearances

    contacts = {
        1: [(0, 1.0), (1, 1.0)],
        2: [(1, 1.0), (2, 1.0)],
    }
    allocation = allocate_cell_speeds(
        contacts,
        {1: 3.0, 2: 1.0},
        {1: 1.0, 2: 1.0},
        cell_count=3,
        idle_speed=2.2,
        minimum_speed=1.0,
        maximum_speed=3.0,
        urgency_gain=1.5,
        idle_regularization=0.03,
        iterations=12,
    )
    assert all(1.0 <= speed <= 3.0 for speed in allocation), allocation
    assert effective_speed(allocation, contacts[1]) > 2.49, allocation
    assert effective_speed(allocation, contacts[2]) < 1.51, allocation

    print("PASS: V7 pairwise profile is bounded and order-preserving.")
    print("transverse:", [round(value, 3) for value in transverse_speeds])
    print("separated:", separated.speed_by_uid)
    print("partial:", [round(value, 3) for value in partial])
    print("deadline delta:", deadline.required_delta_by_pair)
    print("simulated clearances:", [round(value, 3) for value in simulated_clearances])
    print("cell-aware allocation:", [round(value, 3) for value in allocation])


if __name__ == "__main__":
    main()

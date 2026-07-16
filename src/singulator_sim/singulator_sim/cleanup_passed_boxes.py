#!/usr/bin/env python3

import argparse
import re
import subprocess
import threading

from concurrent.futures import ThreadPoolExecutor


WORLD_NAME = "matrix_14x4_stream"

POSE_TOPIC = (
    f"/world/{WORLD_NAME}/pose/info"
)

REMOVE_SERVICE = (
    f"/world/{WORLD_NAME}/remove"
)

BOX_NAME_PATTERN = re.compile(
    r'^box_[A-Za-z0-9_]+$'
)

NUMBER_PATTERN = (
    r"[-+]?"
    r"(?:\d+(?:\.\d*)?|\.\d+)"
    r"(?:[eE][-+]?\d+)?"
)


def extract_field(
    block: str,
    field_name: str,
    default: float = 0.0,
) -> float:
    match = re.search(
        rf"\b{field_name}:\s*({NUMBER_PATTERN})",
        block,
    )

    if match is None:
        return default

    return float(match.group(1))


def parse_pose_block(
    block: str,
) -> tuple[str, float, float] | None:
    name_match = re.search(
        r'name:\s*"([^"]+)"',
        block,
    )

    if name_match is None:
        return None

    name = name_match.group(1)

    position_match = re.search(
        r"position\s*\{(.*?)\}",
        block,
        flags=re.DOTALL,
    )

    if position_match is None:
        return name, 0.0, 0.0

    position_block = position_match.group(1)

    x = extract_field(
        position_block,
        "x",
        default=0.0,
    )

    z = extract_field(
        position_block,
        "z",
        default=0.0,
    )

    return name, x, z


def pose_blocks(stream):
    collecting = False
    depth = 0
    lines = []

    for line in stream:
        stripped = line.strip()

        if not collecting:
            if stripped == "pose {":
                collecting = True
                depth = 1
                lines = [line]

            continue

        lines.append(line)

        depth += line.count("{")
        depth -= line.count("}")

        if depth == 0:
            yield "".join(lines)

            collecting = False
            lines = []


def remove_model(name: str) -> bool:
    request = (
        f'name: "{name}" '
        f'type: MODEL'
    )

    command = [
        "gz",
        "service",
        "-s",
        REMOVE_SERVICE,
        "--reqtype",
        "gz.msgs.Entity",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "2000",
        "--req",
        request,
    ]

    result = subprocess.run(
        command,
        capture_output=True,
        text=True,
        timeout=5,
        check=False,
    )

    success = (
        result.returncode == 0
        and "data: true" in result.stdout.lower()
    )

    if success:
        print(f"[DELETE] {name}")
    else:
        print(f"[DELETE ERROR] {name}")

        if result.stdout.strip():
            print(result.stdout.strip())

        if result.stderr.strip():
            print(result.stderr.strip())

    return success


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Удаление коробок, прошедших "
            "выходной конвейер"
        )
    )

    parser.add_argument(
        "--delete-x",
        type=float,
        default=3.60,
        help="Удалять коробку при X не меньше этого значения.",
    )

    parser.add_argument(
        "--fallen-z",
        type=float,
        default=-0.50,
        help="Удалять упавшие коробки ниже указанного Z.",
    )

    args = parser.parse_args()

    print("Запущена автоматическая очистка.")
    print(f"Pose topic: {POSE_TOPIC}")
    print(f"Remove service: {REMOVE_SERVICE}")
    print(f"Удаление после X >= {args.delete_x:.3f}")
    print(f"Удаление упавших при Z <= {args.fallen_z:.3f}")

    command = [
        "gz",
        "topic",
        "-e",
        "-t",
        POSE_TOPIC,
    ]

    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )

    if process.stdout is None:
        raise RuntimeError(
            "Не удалось открыть вывод gz topic"
        )

    pending = set()
    deleted = set()
    lock = threading.Lock()

    def deletion_finished(
        name: str,
        future,
    ) -> None:
        try:
            success = future.result()
        except Exception as error:
            print(
                f"[DELETE EXCEPTION] {name}: {error}"
            )
            success = False

        with lock:
            pending.discard(name)

            if success:
                deleted.add(name)

    try:
        with ThreadPoolExecutor(max_workers=2) as executor:
            for block in pose_blocks(process.stdout):
                parsed = parse_pose_block(block)

                if parsed is None:
                    continue

                name, x, z = parsed

                if not BOX_NAME_PATTERN.fullmatch(name):
                    continue

                should_delete = (
                    x >= args.delete_x
                    or z <= args.fallen_z
                )

                if not should_delete:
                    continue

                with lock:
                    if name in pending or name in deleted:
                        continue

                    pending.add(name)

                reason = (
                    f"X={x:.3f}"
                    if x >= args.delete_x
                    else f"Z={z:.3f}"
                )

                print(
                    f"[QUEUE DELETE] {name}, {reason}"
                )

                future = executor.submit(
                    remove_model,
                    name,
                )

                future.add_done_callback(
                    lambda completed, model_name=name:
                    deletion_finished(
                        model_name,
                        completed,
                    )
                )

    except KeyboardInterrupt:
        print("\nОчистка остановлена.")

    finally:
        process.terminate()

        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()


if __name__ == "__main__":
    main()

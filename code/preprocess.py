import argparse
import csv
import json
import math
import statistics
import struct
from bisect import bisect_right
from pathlib import Path


MAGIC = b"EP01"
HEADER_STRUCT = struct.Struct("<4sI")
FRAME_HEADER_STRUCT = struct.Struct("<IB")
JOINT_STRUCT = struct.Struct("<Bhhh")
ROOT_JOINT_ID = 0
TARGET_FPS = 30.0
TARGET_DELTA_US = int(round(1_000_000.0 / TARGET_FPS))
SKELETON_BONES = (
    (3, 2),
    (2, 20),
    (20, 1),
    (1, 0),
    (20, 4),
    (4, 5),
    (5, 6),
    (6, 7),
    (20, 8),
    (8, 9),
    (9, 10),
    (10, 11),
    (0, 12),
    (12, 13),
    (13, 14),
    (14, 15),
    (0, 16),
    (16, 17),
    (17, 18),
    (18, 19),
    (7, 21),
    (7, 22),
    (11, 23),
    (11, 24),
)
CONNECTED_LIMB_CHAINS = tuple(
    sorted(
        (
            (joint_a, joint_b, joint_c)
            for joint_a, joint_b in SKELETON_BONES
            for chain_start, joint_c in SKELETON_BONES
            if joint_b == chain_start
        ),
        key=lambda chain: (chain[0], chain[1], chain[2]),
    )
)
LIMB_LENGTH_FALLBACK_BONES = {
    (13, 14): (17, 18),
    (14, 15): (18, 19),
}
DEFAULT_CENTERED_JOINTS = {
    0: (0.0, 0.0, 0.0),
    1: (0.0, 0.26, 0.0),
    2: (0.0, 0.51, 0.0),
    3: (0.0, 0.67, 0.0),
    4: (-0.18, 0.45, 0.0),
    5: (-0.43, 0.28, 0.0),
    6: (-0.65, 0.10, 0.0),
    7: (-0.72, 0.06, 0.0),
    8: (0.18, 0.45, 0.0),
    9: (0.43, 0.28, 0.0),
    10: (0.65, 0.10, 0.0),
    11: (0.72, 0.06, 0.0),
    12: (-0.10, -0.08, 0.0),
    13: (-0.12, -0.45, 0.0),
    14: (-0.12, -0.73, 0.0),
    15: (-0.12, -0.81, 0.04),
    16: (0.10, -0.08, 0.0),
    17: (0.12, -0.45, 0.0),
    18: (0.12, -0.73, 0.0),
    19: (0.12, -0.81, 0.04),
    20: (0.0, 0.45, 0.0),
    21: (-0.76, 0.07, 0.0),
    22: (-0.73, 0.01, 0.0),
    23: (0.76, 0.07, 0.0),
    24: (0.73, 0.01, 0.0),
}


def load_recording(path: Path) -> list[dict]:
    with path.open("rb") as handle:
        header = handle.read(HEADER_STRUCT.size)
        if len(header) != HEADER_STRUCT.size:
            raise ValueError("file too short")

        magic, frame_count = HEADER_STRUCT.unpack(header)
        if magic != MAGIC:
            raise ValueError(f"unexpected magic {magic!r}")

        frames: list[dict] = []
        for _ in range(frame_count):
            frame_header = handle.read(FRAME_HEADER_STRUCT.size)
            if len(frame_header) != FRAME_HEADER_STRUCT.size:
                raise ValueError("unexpected end of file while reading frame header")

            delta_time, joint_count = FRAME_HEADER_STRUCT.unpack(frame_header)
            joints: dict[int, tuple[float, float, float]] = {}

            for _ in range(joint_count):
                joint_bytes = handle.read(JOINT_STRUCT.size)
                if len(joint_bytes) != JOINT_STRUCT.size:
                    raise ValueError("unexpected end of file while reading joint")

                joint_id, x_mm, y_mm, z_mm = JOINT_STRUCT.unpack(joint_bytes)
                joints[joint_id] = (x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0)

            frames.append({"delta_time": delta_time, "joints": joints})

        return frames


def accumulate_timestamps(frames: list[dict]) -> list[int]:
    timestamps_us: list[int] = []
    current_time_us = 0
    for frame in frames:
        current_time_us += frame["delta_time"]
        timestamps_us.append(current_time_us)
    return timestamps_us


def interpolate_joint(
    left_joint: tuple[float, float, float],
    right_joint: tuple[float, float, float],
    alpha: float,
) -> tuple[float, float, float]:
    return (
        left_joint[0] + (right_joint[0] - left_joint[0]) * alpha,
        left_joint[1] + (right_joint[1] - left_joint[1]) * alpha,
        left_joint[2] + (right_joint[2] - left_joint[2]) * alpha,
    )


def resample_frames(frames: list[dict], target_delta_us: int = TARGET_DELTA_US) -> list[dict]:
    if not frames:
        return []

    timestamps_us = accumulate_timestamps(frames)
    total_duration_us = timestamps_us[-1]
    sample_times_us = list(range(0, total_duration_us + 1, target_delta_us))
    if not sample_times_us:
        sample_times_us = [0]

    resampled_frames: list[dict] = []
    for sample_time_us in sample_times_us:
        right_index = bisect_right(timestamps_us, sample_time_us)
        if right_index <= 0:
            left_index = right_frame_index = 0
        elif right_index >= len(frames):
            left_index = right_frame_index = len(frames) - 1
        else:
            left_index = right_index - 1
            right_frame_index = right_index

        left_frame = frames[left_index]
        right_frame = frames[right_frame_index]
        left_time_us = timestamps_us[left_index]
        right_time_us = timestamps_us[right_frame_index]

        if right_frame_index == left_index or right_time_us == left_time_us:
            alpha = 0.0
        else:
            alpha = (sample_time_us - left_time_us) / (right_time_us - left_time_us)

        if right_frame_index == left_index:
            interpolated_joints = dict(left_frame["joints"])
        else:
            joint_ids = sorted(set(left_frame["joints"]) & set(right_frame["joints"]))
            interpolated_joints: dict[int, tuple[float, float, float]] = {}
            for joint_id in joint_ids:
                interpolated_joints[joint_id] = interpolate_joint(
                    left_frame["joints"][joint_id],
                    right_frame["joints"][joint_id],
                    alpha,
                )

        resampled_frames.append(
            {
                "delta_time": 0 if not resampled_frames else target_delta_us,
                "timestamp_us": sample_time_us,
                "joints": interpolated_joints,
            }
        )

    return resampled_frames


def normalize_root_visibility(frames: list[dict]) -> list[dict]:
    valid_indices = [index for index, frame in enumerate(frames) if ROOT_JOINT_ID in frame["joints"]]
    if not valid_indices:
        return []

    first_valid_index = valid_indices[0]
    last_valid_index = valid_indices[-1]
    trimmed_frames = frames[first_valid_index:last_valid_index + 1]

    last_root_position = trimmed_frames[0]["joints"][ROOT_JOINT_ID]
    normalized_frames: list[dict] = []

    for frame in trimmed_frames:
        normalized_joints = dict(frame["joints"])
        if ROOT_JOINT_ID in normalized_joints:
            last_root_position = normalized_joints[ROOT_JOINT_ID]
        else:
            normalized_joints[ROOT_JOINT_ID] = last_root_position

        normalized_frames.append(
            {
                "delta_time": frame["delta_time"],
                "timestamp_us": frame.get("timestamp_us", 0),
                "joints": normalized_joints,
            }
        )

    return normalized_frames


def interpolate_missing_joint_positions(frames: list[dict]) -> list[dict]:
    if not frames:
        return []

    filled_frames = [
        {
            "delta_time": frame["delta_time"],
            "timestamp_us": frame.get("timestamp_us", 0),
            "joints": dict(frame["joints"]),
        }
        for frame in frames
    ]

    for joint_id, default_centered_position in DEFAULT_CENTERED_JOINTS.items():
        known_indices = [
            index
            for index, frame in enumerate(frames)
            if joint_id in frame["joints"]
        ]

        if not known_indices:
            for frame in filled_frames:
                root = frame["joints"].get(ROOT_JOINT_ID, (0.0, 0.0, 0.0))
                frame["joints"][joint_id] = add_vectors(root, default_centered_position)
            continue

        first_known_index = known_indices[0]
        first_known_position = frames[first_known_index]["joints"][joint_id]
        for index in range(0, first_known_index):
            filled_frames[index]["joints"][joint_id] = first_known_position

        for left_index, right_index in zip(known_indices, known_indices[1:]):
            left_position = frames[left_index]["joints"][joint_id]
            right_position = frames[right_index]["joints"][joint_id]
            frame_gap = right_index - left_index

            for index in range(left_index + 1, right_index):
                alpha = (index - left_index) / frame_gap
                filled_frames[index]["joints"][joint_id] = interpolate_joint(left_position, right_position, alpha)

        last_known_index = known_indices[-1]
        last_known_position = frames[last_known_index]["joints"][joint_id]
        for index in range(last_known_index + 1, len(filled_frames)):
            filled_frames[index]["joints"][joint_id] = last_known_position

    return filled_frames


def joint_dict_from_tuple(position: tuple[float, float, float]) -> dict[str, float]:
    return {"x": position[0], "y": position[1], "z": position[2]}


def build_centered_frames(frames: list[dict]) -> list[dict]:
    centered_frames: list[dict] = []
    last_centered_joints: dict[int, dict[str, float]] = {}

    for index, frame in enumerate(frames):
        joints = frame["joints"]
        root = joints.get(ROOT_JOINT_ID, (0.0, 0.0, 0.0))
        centered_joints: dict[str, dict[str, float]] = {}

        for joint_id, default_position in sorted(DEFAULT_CENTERED_JOINTS.items()):
            if joint_id in joints:
                x, y, z = joints[joint_id]
                centered_position = (x - root[0], y - root[1], z - root[2])
                joint_value = joint_dict_from_tuple(centered_position)
            elif joint_id in last_centered_joints:
                joint_value = dict(last_centered_joints[joint_id])
            else:
                joint_value = joint_dict_from_tuple(default_position)

            centered_joints[str(joint_id)] = joint_value
            last_centered_joints[joint_id] = dict(joint_value)

        centered_frames.append(
            {
                "frame_index": index,
                "joints_centered": centered_joints,
            }
        )

    return centered_frames


def preprocess_frames(centered_frames: list[dict]) -> list[dict]:
    processed: list[dict] = []

    for frame in centered_frames:
        limb_angle_quaternions = calculate_limb_angle_quaternions(frame["joints_centered"])
        processed.append(
            {
                "frame_index": frame["frame_index"],
                "limb_angle_quaternions": limb_angle_quaternions,
            }
        )

    enforce_quaternion_continuity(processed)
    return processed


def subtract_vectors(joint_b: dict[str, float], joint_a: dict[str, float]) -> tuple[float, float, float]:
    return (
        joint_b["x"] - joint_a["x"],
        joint_b["y"] - joint_a["y"],
        joint_b["z"] - joint_a["z"],
    )


def vector_length(vector: tuple[float, float, float]) -> float:
    return math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2])


def normalize_vector(vector: tuple[float, float, float]) -> tuple[float, float, float] | None:
    length = vector_length(vector)
    if length <= 1e-9:
        return None

    return (vector[0] / length, vector[1] / length, vector[2] / length)


def cross_vectors(
    vector_a: tuple[float, float, float],
    vector_b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        vector_a[1] * vector_b[2] - vector_a[2] * vector_b[1],
        vector_a[2] * vector_b[0] - vector_a[0] * vector_b[2],
        vector_a[0] * vector_b[1] - vector_a[1] * vector_b[0],
    )


def dot_vectors(vector_a: tuple[float, float, float], vector_b: tuple[float, float, float]) -> float:
    return vector_a[0] * vector_b[0] + vector_a[1] * vector_b[1] + vector_a[2] * vector_b[2]


def choose_orthogonal_axis(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    if abs(vector[0]) < abs(vector[1]):
        candidate = (1.0, 0.0, 0.0)
    else:
        candidate = (0.0, 1.0, 0.0)

    axis = normalize_vector(cross_vectors(vector, candidate))
    if axis is None:
        return (0.0, 0.0, 1.0)
    return axis


def quaternion_between_vectors(
    vector_a: tuple[float, float, float],
    vector_b: tuple[float, float, float],
) -> dict[str, float] | None:
    normalized_a = normalize_vector(vector_a)
    normalized_b = normalize_vector(vector_b)
    if normalized_a is None or normalized_b is None:
        return None

    dot = max(-1.0, min(1.0, dot_vectors(normalized_a, normalized_b)))
    if dot > 1.0 - 1e-9:
        return {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}

    if dot < -1.0 + 1e-9:
        axis = choose_orthogonal_axis(normalized_a)
        return {"w": 0.0, "x": axis[0], "y": axis[1], "z": axis[2]}

    cross = cross_vectors(normalized_a, normalized_b)
    quaternion = (1.0 + dot, cross[0], cross[1], cross[2])
    quaternion_length = math.sqrt(sum(component * component for component in quaternion))

    return {
        "w": quaternion[0] / quaternion_length,
        "x": quaternion[1] / quaternion_length,
        "y": quaternion[2] / quaternion_length,
        "z": quaternion[3] / quaternion_length,
    }


def calculate_limb_angle_quaternions(centered_joints: dict[str, dict[str, float]]) -> dict[str, dict[str, float]]:
    quaternions: dict[str, dict[str, float]] = {}

    for joint_a, joint_b, joint_c in CONNECTED_LIMB_CHAINS:
        joint_a_value = centered_joints.get(str(joint_a))
        joint_b_value = centered_joints.get(str(joint_b))
        joint_c_value = centered_joints.get(str(joint_c))
        if joint_a_value is None or joint_b_value is None or joint_c_value is None:
            continue

        parent_limb = subtract_vectors(joint_b_value, joint_a_value)
        child_limb = subtract_vectors(joint_c_value, joint_b_value)
        quaternion = quaternion_between_vectors(parent_limb, child_limb)
        if quaternion is None:
            continue

        quaternions[f"{joint_a}-{joint_b}-{joint_c}"] = quaternion

    return quaternions


def quaternion_dot(left: dict[str, float], right: dict[str, float]) -> float:
    return left["w"] * right["w"] + left["x"] * right["x"] + left["y"] * right["y"] + left["z"] * right["z"]


def negate_quaternion(quaternion: dict[str, float]) -> dict[str, float]:
    return {
        "w": -quaternion["w"],
        "x": -quaternion["x"],
        "y": -quaternion["y"],
        "z": -quaternion["z"],
    }


def enforce_quaternion_continuity(processed_frames: list[dict]) -> None:
    previous_quaternions: dict[str, dict[str, float]] = {}

    for frame in processed_frames:
        quaternions = frame["limb_angle_quaternions"]
        for chain, quaternion in quaternions.items():
            previous_quaternion = previous_quaternions.get(chain)
            if previous_quaternion is not None and quaternion_dot(previous_quaternion, quaternion) < 0.0:
                quaternion = negate_quaternion(quaternion)
                quaternions[chain] = quaternion

            previous_quaternions[chain] = quaternion


def parse_quaternion(value: str) -> dict[str, float]:
    w, x, y, z = (float(component) for component in value.split())
    return {"w": w, "x": x, "y": y, "z": z}


def quaternion_conjugate(quaternion: dict[str, float]) -> dict[str, float]:
    return {
        "w": quaternion["w"],
        "x": -quaternion["x"],
        "y": -quaternion["y"],
        "z": -quaternion["z"],
    }


def multiply_quaternions(left: dict[str, float], right: dict[str, float]) -> dict[str, float]:
    return {
        "w": left["w"] * right["w"] - left["x"] * right["x"] - left["y"] * right["y"] - left["z"] * right["z"],
        "x": left["w"] * right["x"] + left["x"] * right["w"] + left["y"] * right["z"] - left["z"] * right["y"],
        "y": left["w"] * right["y"] - left["x"] * right["z"] + left["y"] * right["w"] + left["z"] * right["x"],
        "z": left["w"] * right["z"] + left["x"] * right["y"] - left["y"] * right["x"] + left["z"] * right["w"],
    }


def rotate_vector(vector: tuple[float, float, float], quaternion: dict[str, float]) -> tuple[float, float, float]:
    vector_quaternion = {"w": 0.0, "x": vector[0], "y": vector[1], "z": vector[2]}
    rotated = multiply_quaternions(
        multiply_quaternions(quaternion, vector_quaternion),
        quaternion_conjugate(quaternion),
    )
    return (rotated["x"], rotated["y"], rotated["z"])


def scale_vector(vector: tuple[float, float, float], length: float) -> tuple[float, float, float]:
    normalized = normalize_vector(vector)
    if normalized is None:
        return (0.0, length, 0.0)
    return (normalized[0] * length, normalized[1] * length, normalized[2] * length)


def add_vectors(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (left[0] + right[0], left[1] + right[1], left[2] + right[2])


def subtract_position_vectors(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])


def load_limb_lengths_csv(path: Path) -> dict[tuple[int, int], float]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        lengths: dict[tuple[int, int], float] = {}
        for row in reader:
            length_value = row.get("median_length_mm") or row.get("mean_length_mm") or ""
            if not length_value:
                continue

            lengths[(int(row["limb_start"]), int(row["limb_end"]))] = float(length_value) / 1000.0

        return lengths


def load_quaternion_csv(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        rows: list[dict] = []
        for row in reader:
            rows.append(
                {
                    "frame_index": int(row["frame_index"]),
                    "limb_angle_quaternions": {
                        column: parse_quaternion(value)
                        for column, value in row.items()
                        if column != "frame_index"
                    },
                }
            )
        return rows


def limb_length_for(lengths: dict[tuple[int, int], float], joint_a: int, joint_b: int) -> float:
    return lengths.get((joint_a, joint_b), lengths.get((joint_b, joint_a), 0.1))


def default_direction(joint_a: int, joint_b: int) -> tuple[float, float, float]:
    return subtract_position_vectors(DEFAULT_CENTERED_JOINTS[joint_b], DEFAULT_CENTERED_JOINTS[joint_a])


def seed_root_neighbors(lengths: dict[tuple[int, int], float]) -> dict[int, tuple[float, float, float]]:
    positions: dict[int, tuple[float, float, float]] = {ROOT_JOINT_ID: (0.0, 0.0, 0.0)}
    for joint_a, joint_b in SKELETON_BONES:
        if joint_a == ROOT_JOINT_ID:
            neighbor = joint_b
            direction = default_direction(ROOT_JOINT_ID, neighbor)
        elif joint_b == ROOT_JOINT_ID:
            neighbor = joint_a
            direction = default_direction(ROOT_JOINT_ID, neighbor)
        else:
            continue

        positions[neighbor] = scale_vector(direction, limb_length_for(lengths, joint_a, joint_b))

    return positions


def fill_remaining_default_positions(
    positions: dict[int, tuple[float, float, float]],
    lengths: dict[tuple[int, int], float],
) -> None:
    changed = True
    while changed:
        changed = False
        for joint_a, joint_b in SKELETON_BONES:
            if joint_a in positions and joint_b not in positions:
                direction = default_direction(joint_a, joint_b)
                positions[joint_b] = add_vectors(
                    positions[joint_a],
                    scale_vector(direction, limb_length_for(lengths, joint_a, joint_b)),
                )
                changed = True
            elif joint_b in positions and joint_a not in positions:
                direction = default_direction(joint_b, joint_a)
                positions[joint_a] = add_vectors(
                    positions[joint_b],
                    scale_vector(direction, limb_length_for(lengths, joint_a, joint_b)),
                )
                changed = True


def reconstruct_positions_from_quaternions(
    quaternions: dict[str, dict[str, float]],
    lengths: dict[tuple[int, int], float],
) -> dict[int, tuple[float, float, float]]:
    positions = seed_root_neighbors(lengths)

    changed = True
    while changed:
        changed = False
        for joint_a, joint_b, joint_c in CONNECTED_LIMB_CHAINS:
            quaternion = quaternions[f"{joint_a}-{joint_b}-{joint_c}"]

            if joint_a in positions and joint_b in positions and joint_c not in positions:
                parent_vector = subtract_position_vectors(positions[joint_b], positions[joint_a])
                child_direction = rotate_vector(parent_vector, quaternion)
                positions[joint_c] = add_vectors(
                    positions[joint_b],
                    scale_vector(child_direction, limb_length_for(lengths, joint_b, joint_c)),
                )
                changed = True

            if joint_b in positions and joint_c in positions and joint_a not in positions:
                child_vector = subtract_position_vectors(positions[joint_c], positions[joint_b])
                parent_direction = rotate_vector(child_vector, quaternion_conjugate(quaternion))
                positions[joint_a] = subtract_position_vectors(
                    positions[joint_b],
                    scale_vector(parent_direction, limb_length_for(lengths, joint_a, joint_b)),
                )
                changed = True

    fill_remaining_default_positions(positions, lengths)
    return positions


def reconstruct_frames_from_csv(quaternion_csv_path: Path, limb_lengths_csv_path: Path) -> list[dict]:
    lengths = load_limb_lengths_csv(limb_lengths_csv_path)
    quaternion_frames = load_quaternion_csv(quaternion_csv_path)
    return [
        {
            "frame_index": frame["frame_index"],
            "joints": reconstruct_positions_from_quaternions(frame["limb_angle_quaternions"], lengths),
        }
        for frame in quaternion_frames
    ]


def invert_processed_frame(frame: dict) -> dict[int, tuple[float, float, float]]:
    root = frame["root_position"]
    absolute_joints: dict[int, tuple[float, float, float]] = {}
    for joint_id, joint_value in frame["joints_centered"].items():
        absolute_joints[int(joint_id)] = (
            root["x"] + joint_value["x"],
            root["y"] + joint_value["y"],
            root["z"] + joint_value["z"],
        )
    return absolute_joints


def collect_limb_length_measurements(processed_frames: list[dict]) -> dict[tuple[int, int], list[float]]:
    measurements: dict[tuple[int, int], list[float]] = {}

    for joint_a, joint_b in SKELETON_BONES:
        lengths: list[float] = []

        for frame in processed_frames:
            joints = frame["joints_centered"]
            joint_a_value = joints.get(str(joint_a))
            joint_b_value = joints.get(str(joint_b))
            if joint_a_value is None or joint_b_value is None:
                continue

            dx = joint_a_value["x"] - joint_b_value["x"]
            dy = joint_a_value["y"] - joint_b_value["y"]
            dz = joint_a_value["z"] - joint_b_value["z"]
            lengths.append(math.sqrt(dx * dx + dy * dy + dz * dz))

        measurements[(joint_a, joint_b)] = lengths

    return measurements


def calculate_limb_lengths(processed_frames: list[dict]) -> list[dict]:
    limb_measurements = collect_limb_length_measurements(processed_frames)
    limb_lengths: list[dict] = []

    for joint_a, joint_b in sorted(SKELETON_BONES):
        lengths = limb_measurements[(joint_a, joint_b)]
        fallback_bone = LIMB_LENGTH_FALLBACK_BONES.get((joint_a, joint_b))
        if not lengths and fallback_bone is not None:
            lengths = limb_measurements.get(fallback_bone, [])

        if not lengths:
            limb_lengths.append(
                {
                    "limb_start": joint_a,
                    "limb_end": joint_b,
                    "median_length_mm": "",
                }
            )
            continue

        limb_lengths.append(
            {
                "limb_start": joint_a,
                "limb_end": joint_b,
                "median_length_mm": statistics.median(lengths) * 1000.0,
            }
        )

    return limb_lengths


def build_processed_path(recording_path: Path, processed_directory: Path) -> Path:
    processed_directory.mkdir(parents=True, exist_ok=True)
    sample_id = recording_path.stem.removeprefix("recording_")
    return processed_directory / f"processed_{sample_id}.csv"


def build_limb_lengths_path(recording_path: Path, data_directory: Path) -> Path:
    sample_id = recording_path.stem.removeprefix("recording_")
    return data_directory / f"limb_lengths_{sample_id}.csv"


def save_json(data: object, output_path: Path) -> None:
    if output_path.exists():
        output_path.unlink()

    with output_path.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2)


def format_quaternion(quaternion: dict[str, float]) -> str:
    return f"{quaternion['w']} {quaternion['x']} {quaternion['y']} {quaternion['z']}"


def save_processed_csv(processed_frames: list[dict], output_path: Path) -> None:
    if output_path.exists():
        output_path.unlink()

    limb_columns = [f"{joint_a}-{joint_b}-{joint_c}" for joint_a, joint_b, joint_c in CONNECTED_LIMB_CHAINS]
    fieldnames = ["frame_index", *limb_columns]

    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()

        for frame in processed_frames:
            quaternions = frame["limb_angle_quaternions"]
            row = {"frame_index": frame["frame_index"]}
            for column in limb_columns:
                row[column] = format_quaternion(quaternions[column])
            writer.writerow(row)


def save_limb_lengths_csv(limb_lengths: list[dict], output_path: Path) -> None:
    if output_path.exists():
        output_path.unlink()

    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=("limb_start", "limb_end", "median_length_mm"))
        writer.writeheader()
        writer.writerows(limb_lengths)


def print_recording_summary(frames: list[dict], processed_path: Path) -> None:
    print(f"Loaded {len(frames)} raw frames")
    if frames:
        print(
            f"First frame delta: {frames[0]['delta_time']} us, "
            f"tracked joints: {len(frames[0]['joints'])}"
        )
    print(f"Wrote processed output to {processed_path}")


def print_limb_lengths_summary(limb_lengths_path: Path) -> None:
    print(f"Wrote limb lengths to {limb_lengths_path}")


def main() -> int:
    script_directory = Path(__file__).resolve().parent
    data_directory = script_directory / "data"
    raw_directory = data_directory / "recordings" / "raw"
    processed_directory = data_directory / "recordings" / "processed"

    parser = argparse.ArgumentParser(description="Read raw EMG pose recordings and write processed features.")
    parser.add_argument(
        "recording",
        nargs="?",
        default="0",
        help="Raw recording path or numeric sample id like 0, 1, 2",
    )
    args = parser.parse_args()

    if args.recording.isdigit():
        recording_path = raw_directory / f"recording_{args.recording}.bin"
    else:
        recording_path = Path(args.recording)

    raw_frames = load_recording(recording_path)
    resampled_frames = resample_frames(raw_frames)
    normalized_frames = normalize_root_visibility(resampled_frames)
    filled_frames = interpolate_missing_joint_positions(normalized_frames)
    centered_frames = build_centered_frames(filled_frames)
    processed_frames = preprocess_frames(centered_frames)
    limb_lengths = calculate_limb_lengths(centered_frames)

    processed_path = build_processed_path(recording_path, processed_directory)
    limb_lengths_path = build_limb_lengths_path(recording_path, data_directory)
    save_processed_csv(processed_frames, processed_path)
    save_limb_lengths_csv(limb_lengths, limb_lengths_path)
    print_recording_summary(filled_frames, processed_path)
    print_limb_lengths_summary(limb_lengths_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

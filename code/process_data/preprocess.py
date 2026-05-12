import argparse
import csv
import math
import struct
from bisect import bisect_right
from dataclasses import dataclass
from pathlib import Path

try:
    from process_data.filters import (
        filter_normalized_pose_jitter,
        format_normalized_limit_values,
        parse_normalized_limit_values,
    )
except ModuleNotFoundError:
    from filters import (
        filter_normalized_pose_jitter,
        format_normalized_limit_values,
        parse_normalized_limit_values,
    )


# This module owns the raw-recording -> processed-pose pipeline and also
# contains the inverse reconstruction helpers used by visualization/evaluation.
MAGIC = b"EP01"
HEADER_STRUCT = struct.Struct("<4sI")
FRAME_HEADER_STRUCT = struct.Struct("<IB")
JOINT_STRUCT = struct.Struct("<Bhhh")
ROOT_JOINT_ID = 0
TARGET_FPS = 60.0
KINECT_TICKS_PER_SECOND = 10_000_000
TARGET_DELTA_TICKS = int(round(KINECT_TICKS_PER_SECOND / TARGET_FPS))
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


def build_outward_skeleton_bones(root_joint_id: int) -> tuple[tuple[int, int], ...]:
    adjacency: dict[int, set[int]] = {}
    for joint_a, joint_b in SKELETON_BONES:
        adjacency.setdefault(joint_a, set()).add(joint_b)
        adjacency.setdefault(joint_b, set()).add(joint_a)

    directed_bones: list[tuple[int, int]] = []
    visited = {root_joint_id}
    queue = [root_joint_id]
    while queue:
        joint_a = queue.pop(0)
        for joint_b in sorted(adjacency.get(joint_a, ())):
            if joint_b in visited:
                continue
            visited.add(joint_b)
            directed_bones.append((joint_a, joint_b))
            queue.append(joint_b)

    return tuple(directed_bones)


OUTWARD_SKELETON_BONES = build_outward_skeleton_bones(ROOT_JOINT_ID)
ROOT_LIMB_REFERENCE_DIRECTIONS = {
    (0, 1): (0.0, 1.0, 0.0),
    (0, 12): (-1.0, 0.0, 0.0),
    (0, 16): (1.0, 0.0, 0.0),
}
ROOT_LIMB_COLUMNS = tuple(
    f"ROOT-{joint_a}-{joint_b}"
    for joint_a, joint_b in OUTWARD_SKELETON_BONES
    if joint_a == ROOT_JOINT_ID
)
CONNECTED_LIMB_CHAINS = tuple(
    (joint_a, joint_b, joint_c)
    for joint_a, joint_b in OUTWARD_SKELETON_BONES
    for chain_start, joint_c in OUTWARD_SKELETON_BONES
    if joint_b == chain_start
)
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
T_POSE_CENTERED_JOINTS = {
    0: (0.0, 0.0, 0.0),
    1: (0.0, 0.26, 0.0),
    2: (0.0, 0.51, 0.0),
    3: (0.0, 0.67, 0.0),
    4: (-0.22, 0.45, 0.0),
    5: (-0.52, 0.45, 0.0),
    6: (-0.78, 0.45, 0.0),
    7: (-0.88, 0.45, 0.0),
    8: (0.22, 0.45, 0.0),
    9: (0.52, 0.45, 0.0),
    10: (0.78, 0.45, 0.0),
    11: (0.88, 0.45, 0.0),
    12: (-0.10, -0.08, 0.0),
    13: (-0.10, -0.45, 0.0),
    14: (-0.10, -0.73, 0.0),
    15: (-0.10, -0.73, 0.10),
    16: (0.10, -0.08, 0.0),
    17: (0.10, -0.45, 0.0),
    18: (0.10, -0.73, 0.0),
    19: (0.10, -0.73, 0.10),
    20: (0.0, 0.45, 0.0),
    21: (-0.98, 0.45, 0.0),
    22: (-0.88, 0.43, 0.08),
    23: (0.98, 0.45, 0.0),
    24: (0.88, 0.43, 0.08),
}
FOOT_JOINT_IDS = {15, 19}
REQUIRED_VISIBLE_JOINT_IDS = set(DEFAULT_CENTERED_JOINTS.keys()) - FOOT_JOINT_IDS
MAX_JOINT_JUMP_M = 0.12
JITTER_FILTER_EXCLUDED_JOINT_IDS = {21, 22, 23, 24}


@dataclass(frozen=True)
class JointLimit:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_z: float
    max_z: float


@dataclass
class ReconstructionDiagnostics:
    leg_side_separation_fixes: int = 0
    leg_downward_orientation_fixes: int = 0
    foot_forward_orientation_fixes: int = 0


def load_recording(path: Path) -> list[dict]:
    # The binary format is a compact sequence of tracked joints per frame:
    # [delta_time, tracked_joint_count, repeated (joint_id, x_mm, y_mm, z_mm)].
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
    timestamps: list[int] = []
    current_time = 0
    for frame in frames:
        current_time += frame["delta_time"]
        timestamps.append(current_time)
    return timestamps


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


def resample_frames(frames: list[dict], target_delta_ticks: int = TARGET_DELTA_TICKS) -> list[dict]:
    # Resample the irregular Kinect capture times onto a fixed-rate timeline so
    # later stages can assume a stable FPS.
    if not frames:
        return []

    timestamps = accumulate_timestamps(frames)
    total_duration = timestamps[-1]
    sample_times = list(range(0, total_duration + 1, target_delta_ticks))
    if not sample_times:
        sample_times = [0]

    resampled_frames: list[dict] = []
    for sample_time in sample_times:
        right_index = bisect_right(timestamps, sample_time)
        if right_index <= 0:
            left_index = right_frame_index = 0
        elif right_index >= len(frames):
            left_index = right_frame_index = len(frames) - 1
        else:
            left_index = right_index - 1
            right_frame_index = right_index

        left_frame = frames[left_index]
        right_frame = frames[right_frame_index]
        left_time = timestamps[left_index]
        right_time = timestamps[right_frame_index]

        if right_frame_index == left_index or right_time == left_time:
            alpha = 0.0
        else:
            alpha = (sample_time - left_time) / (right_time - left_time)

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
                "delta_time": 0 if not resampled_frames else target_delta_ticks,
                "timestamp_ticks": sample_time,
                "joints": interpolated_joints,
            }
        )

    return resampled_frames


def normalize_root_visibility(frames: list[dict]) -> list[dict]:
    # Trim away unusable start/end regions until every required non-foot joint
    # is visible, then keep the root joint alive across short dropouts.
    valid_indices = [
        index
        for index, frame in enumerate(frames)
        if REQUIRED_VISIBLE_JOINT_IDS.issubset(frame["joints"].keys())
    ]
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
                "timestamp_ticks": frame.get("timestamp_ticks", 0),
                "joints": normalized_joints,
            }
        )

    return normalized_frames


def interpolate_missing_joint_positions(frames: list[dict]) -> list[dict]:
    # Fill joint gaps inside the kept clip. This prevents downstream pose
    # conversion from treating tracking dropouts as real motion.
    if not frames:
        return []

    filled_frames = [
        {
            "delta_time": frame["delta_time"],
            "timestamp_ticks": frame.get("timestamp_ticks", 0),
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
    # Express every joint relative to the root so the representation focuses on
    # pose rather than absolute global position in camera space.
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


def filter_joint_jitter(
    centered_frames: list[dict],
    max_joint_jump_m: float = MAX_JOINT_JUMP_M,
) -> list[dict]:
    # Reject isolated one-frame body-joint spikes before quaternion conversion.
    # Use original neighboring frames instead of the previous filtered frame, so
    # a temporary tracking collapse can recover instead of freezing the limb.
    # Hand tip/thumb joints are excluded so open/close motion is preserved.
    if not centered_frames:
        return []

    filtered_frames: list[dict] = []

    for frame_index, frame in enumerate(centered_frames):
        previous_frame = centered_frames[frame_index - 1] if frame_index > 0 else None
        next_frame = centered_frames[frame_index + 1] if frame_index + 1 < len(centered_frames) else None
        previous_joints = previous_frame["joints_centered"] if previous_frame is not None else {}
        next_joints = next_frame["joints_centered"] if next_frame is not None else {}
        filtered_joints: dict[str, dict[str, float]] = {}

        for joint_id_text, joint_value in frame["joints_centered"].items():
            joint_id = int(joint_id_text)
            previous_value = previous_joints.get(joint_id_text)
            next_value = next_joints.get(joint_id_text)
            if previous_value is None or next_value is None or joint_id in JITTER_FILTER_EXCLUDED_JOINT_IDS:
                filtered_joints[joint_id_text] = dict(joint_value)
                continue

            current_position = (joint_value["x"], joint_value["y"], joint_value["z"])
            previous_position = (previous_value["x"], previous_value["y"], previous_value["z"])
            next_position = (next_value["x"], next_value["y"], next_value["z"])
            is_isolated_spike = (
                joint_distance(current_position, previous_position) > max_joint_jump_m
                and joint_distance(current_position, next_position) > max_joint_jump_m
                and joint_distance(previous_position, next_position) <= max_joint_jump_m
            )
            if is_isolated_spike:
                filtered_joints[joint_id_text] = dict(previous_value)
            else:
                filtered_joints[joint_id_text] = dict(joint_value)

        filtered_frames.append(
            {
                "frame_index": frame["frame_index"],
                "joints_centered": filtered_joints,
            }
        )

    return filtered_frames


def preprocess_frames(centered_frames: list[dict]) -> list[dict]:
    # Convert centered joint positions into per-chain relative rotations.
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


def joint_distance(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> float:
    return vector_length(
        (
            left[0] - right[0],
            left[1] - right[1],
            left[2] - right[2],
        )
    )


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


def calculate_absolute_limb_angle_quaternions(
    centered_joints: dict[str, dict[str, float]],
) -> dict[str, dict[str, float]]:
    # Root-limb quaternions map a neutral body axis onto the current root limb.
    # Other quaternions map a parent limb direction onto its child limb direction.
    quaternions: dict[str, dict[str, float]] = {}

    for joint_a, joint_b in OUTWARD_SKELETON_BONES:
        if joint_a != ROOT_JOINT_ID:
            continue

        joint_a_value = centered_joints.get(str(joint_a))
        joint_b_value = centered_joints.get(str(joint_b))
        if joint_a_value is None or joint_b_value is None:
            continue

        reference_direction = ROOT_LIMB_REFERENCE_DIRECTIONS[(joint_a, joint_b)]
        current_limb = subtract_vectors(joint_b_value, joint_a_value)
        quaternion = quaternion_between_vectors(reference_direction, current_limb)
        if quaternion is None:
            continue

        quaternions[f"ROOT-{joint_a}-{joint_b}"] = quaternion

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


def calculate_limb_angle_quaternions(centered_joints: dict[str, dict[str, float]]) -> dict[str, dict[str, float]]:
    # Store every limb relation relative to the T-pose reference. In this space
    # the T-pose with palms down is the zero/identity rotation for every chain.
    absolute_quaternions = calculate_absolute_limb_angle_quaternions(centered_joints)
    reference_quaternions = t_pose_reference_quaternions()
    relative_quaternions: dict[str, dict[str, float]] = {}

    for chain, quaternion in absolute_quaternions.items():
        reference_quaternion = reference_quaternions.get(chain)
        if reference_quaternion is None:
            relative_quaternions[chain] = quaternion
            continue

        relative_quaternions[chain] = multiply_quaternions(
            quaternion,
            quaternion_conjugate(reference_quaternion),
        )

    return relative_quaternions


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
    # q and -q represent the same rotation. Keep signs temporally consistent so
    # frame-to-frame comparisons do not see artificial flips.
    previous_quaternions: dict[str, dict[str, float]] = {}

    for frame in processed_frames:
        quaternions = frame["limb_angle_quaternions"]
        for chain, quaternion in quaternions.items():
            previous_quaternion = previous_quaternions.get(chain)
            if previous_quaternion is not None and quaternion_dot(previous_quaternion, quaternion) < 0.0:
                quaternion = negate_quaternion(quaternion)
                quaternions[chain] = quaternion

            previous_quaternions[chain] = quaternion


def load_joint_limits(path: Path) -> dict[str, JointLimit]:
    if not path.exists():
        raise FileNotFoundError(f"Joint limits not found: {path}")

    chain_limits: dict[str, JointLimit] = {}
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            chain_limits[row["chain"]] = JointLimit(
                min_x=float(row["min_x"]),
                max_x=float(row["max_x"]),
                min_y=float(row["min_y"]),
                max_y=float(row["max_y"]),
                min_z=float(row["min_z"]),
                max_z=float(row["max_z"]),
            )
    return chain_limits


def quaternion_to_axis_angle(quaternion: dict[str, float]) -> tuple[tuple[float, float, float], float]:
    length = math.sqrt(sum(quaternion[component] * quaternion[component] for component in ("w", "x", "y", "z")))
    if length <= 1e-12:
        return (1.0, 0.0, 0.0), 0.0

    normalized = {component: quaternion[component] / length for component in ("w", "x", "y", "z")}
    w = max(-1.0, min(1.0, normalized["w"]))
    angle = 2.0 * math.acos(w)
    sin_half_angle = math.sqrt(max(0.0, 1.0 - w * w))
    if sin_half_angle <= 1e-12:
        return (1.0, 0.0, 0.0), 0.0

    axis = (
        normalized["x"] / sin_half_angle,
        normalized["y"] / sin_half_angle,
        normalized["z"] / sin_half_angle,
    )
    if angle > math.pi:
        angle = (2.0 * math.pi) - angle
        axis = (-axis[0], -axis[1], -axis[2])
    return axis, angle


def quaternion_to_rotation_vector_degrees(quaternion: dict[str, float]) -> tuple[float, float, float]:
    axis, angle = quaternion_to_axis_angle(quaternion)
    angle_degrees = math.degrees(angle)
    return (axis[0] * angle_degrees, axis[1] * angle_degrees, axis[2] * angle_degrees)


def quaternion_from_axis_angle(axis: tuple[float, float, float], angle: float) -> dict[str, float]:
    normalized_axis = normalize_vector(axis)
    if normalized_axis is None:
        return {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}

    half_angle = angle / 2.0
    sin_half_angle = math.sin(half_angle)
    return {
        "w": math.cos(half_angle),
        "x": normalized_axis[0] * sin_half_angle,
        "y": normalized_axis[1] * sin_half_angle,
        "z": normalized_axis[2] * sin_half_angle,
    }


def rotation_vector_degrees_to_quaternion(rotation_vector: tuple[float, float, float]) -> dict[str, float]:
    angle_degrees = math.sqrt(sum(component * component for component in rotation_vector))
    if angle_degrees <= 1e-12:
        return {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}

    axis = tuple(component / angle_degrees for component in rotation_vector)
    return quaternion_from_axis_angle(axis, math.radians(angle_degrees))


def normalize_value(value: float, minimum: float, maximum: float) -> float:
    if abs(maximum - minimum) <= 1e-12:
        return 0.5
    return max(0.0, min(1.0, (value - minimum) / (maximum - minimum)))


def denormalize_value(value: float, minimum: float, maximum: float) -> float:
    clamped = max(0.0, min(1.0, value))
    return minimum + (clamped * (maximum - minimum))


def quaternion_to_normalized_limit_values(
    quaternion: dict[str, float],
    limit: JointLimit,
) -> tuple[float, float, float]:
    x, y, z = quaternion_to_rotation_vector_degrees(quaternion)
    return (
        normalize_value(x, limit.min_x, limit.max_x),
        normalize_value(y, limit.min_y, limit.max_y),
        normalize_value(z, limit.min_z, limit.max_z),
    )


def normalized_limit_values_to_quaternion(
    values: tuple[float, float, float],
    limit: JointLimit,
) -> dict[str, float]:
    rotation_vector = (
        denormalize_value(values[0], limit.min_x, limit.max_x),
        denormalize_value(values[1], limit.min_y, limit.max_y),
        denormalize_value(values[2], limit.min_z, limit.max_z),
    )
    return rotation_vector_degrees_to_quaternion(rotation_vector)


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


_T_POSE_REFERENCE_QUATERNIONS: dict[str, dict[str, float]] | None = None


def t_pose_reference_quaternions() -> dict[str, dict[str, float]]:
    global _T_POSE_REFERENCE_QUATERNIONS
    if _T_POSE_REFERENCE_QUATERNIONS is None:
        centered_joints = {
            str(joint_id): joint_dict_from_tuple(position)
            for joint_id, position in T_POSE_CENTERED_JOINTS.items()
        }
        _T_POSE_REFERENCE_QUATERNIONS = calculate_absolute_limb_angle_quaternions(centered_joints)
    return _T_POSE_REFERENCE_QUATERNIONS


def compose_with_t_pose_reference(chain: str, relative_quaternion: dict[str, float]) -> dict[str, float]:
    reference_quaternion = t_pose_reference_quaternions().get(chain)
    if reference_quaternion is None:
        return relative_quaternion
    return multiply_quaternions(relative_quaternion, reference_quaternion)


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
    # Limb lengths are treated as shared person-level metadata loaded from the
    # dataset-level CSV rather than inferred per recording.
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        lengths: dict[tuple[int, int], float] = {}
        for row in reader:
            length_value = row.get("length_mm") or row.get("median_length_mm") or row.get("mean_length_mm") or ""
            if not length_value:
                continue

            lengths[(int(row["limb_start"]), int(row["limb_end"]))] = float(length_value) / 1000.0

        return lengths


def parse_pose_cell(
    column: str,
    value: str,
    chain_limits: dict[str, JointLimit],
) -> dict[str, float]:
    limit = chain_limits.get(column)
    if limit is None:
        raise ValueError(f"Normalized pose column {column!r} has no joint limit entry")
    return normalized_limit_values_to_quaternion(parse_normalized_limit_values(value), limit)


def load_pose_csv(path: Path, joint_limits_path: Path | None = None) -> list[dict]:
    if joint_limits_path is None:
        joint_limits_path = Path(__file__).resolve().parent.parent / "data" / "joint_limits.csv"
    chain_limits = load_joint_limits(joint_limits_path)

    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        rows: list[dict] = []
        for row in reader:
            rows.append(
                {
                    "frame_index": int(row["frame_index"]),
                    "limb_angle_quaternions": {
                        column: parse_pose_cell(column, value, chain_limits)
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


def seed_root_limb_positions(
    quaternions: dict[str, dict[str, float]],
    lengths: dict[tuple[int, int], float],
) -> dict[int, tuple[float, float, float]]:
    # Root-child limbs are oriented from explicit neutral body axes:
    # spine up, left hip left, right hip right.
    positions: dict[int, tuple[float, float, float]] = {ROOT_JOINT_ID: (0.0, 0.0, 0.0)}
    for joint_a, joint_b in OUTWARD_SKELETON_BONES:
        if joint_a != ROOT_JOINT_ID:
            continue

        column = f"ROOT-{joint_a}-{joint_b}"
        reference_direction = ROOT_LIMB_REFERENCE_DIRECTIONS[(joint_a, joint_b)]
        relative_quaternion = quaternions.get(column, {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0})
        quaternion = compose_with_t_pose_reference(column, relative_quaternion)
        direction = rotate_vector(reference_direction, quaternion)
        positions[joint_b] = add_vectors(
            positions[joint_a],
            scale_vector(direction, limb_length_for(lengths, joint_a, joint_b)),
        )

    return positions


def fill_remaining_default_positions(
    positions: dict[int, tuple[float, float, float]],
    lengths: dict[tuple[int, int], float],
) -> None:
    changed = True
    while changed:
        changed = False
        for joint_a, joint_b in OUTWARD_SKELETON_BONES:
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


def enforce_leg_side_separation(
    positions: dict[int, tuple[float, float, float]],
    diagnostics: ReconstructionDiagnostics | None = None,
) -> None:
    # A simple post-pass that keeps reconstructed left/right legs on their
    # expected sides when the local-angle representation becomes ambiguous.
    for joint_id in (12, 13, 14, 15):
        if joint_id in positions and positions[joint_id][0] > 0.0:
            x, y, z = positions[joint_id]
            positions[joint_id] = (-abs(x), y, z)
            if diagnostics is not None:
                diagnostics.leg_side_separation_fixes += 1

    for joint_id in (16, 17, 18, 19):
        if joint_id in positions and positions[joint_id][0] < 0.0:
            x, y, z = positions[joint_id]
            positions[joint_id] = (abs(x), y, z)
            if diagnostics is not None:
                diagnostics.leg_side_separation_fixes += 1


def enforce_leg_downward_orientation(
    positions: dict[int, tuple[float, float, float]],
    diagnostics: ReconstructionDiagnostics | None = None,
) -> None:
    # Local limb-angle reconstruction can occasionally choose the mirrored leg
    # solution. Keep each leg branch meaningfully below its hip when that happens.
    for hip_id, knee_id, ankle_id, foot_id in ((12, 13, 14, 15), (16, 17, 18, 19)):
        hip_position = positions.get(hip_id)
        knee_position = positions.get(knee_id)
        if hip_position is None or knee_position is None:
            continue

        hip_y = hip_position[1]
        hip_to_knee_length = joint_distance(hip_position, knee_position)
        max_knee_y = hip_y - (0.25 * hip_to_knee_length)
        if knee_position[1] <= max_knee_y:
            continue

        y_delta = max_knee_y - knee_position[1]
        if diagnostics is not None:
            diagnostics.leg_downward_orientation_fixes += 1
        for joint_id in (knee_id, ankle_id, foot_id):
            position = positions.get(joint_id)
            if position is None:
                continue
            x, y, z = position
            positions[joint_id] = (x, y + y_delta, z)


def enforce_foot_forward_orientation(
    positions: dict[int, tuple[float, float, float]],
    lengths: dict[tuple[int, int], float],
    diagnostics: ReconstructionDiagnostics | None = None,
) -> None:
    # Kinect foot tracking is noisy and can flip the short ankle->foot segment
    # up/down. Keep feet mostly level with the ankle and pointing forward.
    for ankle_id, foot_id in ((14, 15), (18, 19)):
        ankle_position = positions.get(ankle_id)
        foot_position = positions.get(foot_id)
        if ankle_position is None or foot_position is None:
            continue

        foot_length = limb_length_for(lengths, ankle_id, foot_id)
        dx = foot_position[0] - ankle_position[0]
        dz = foot_position[2] - ankle_position[2]
        horizontal_length = math.sqrt((dx * dx) + (dz * dz))
        if horizontal_length <= 1e-9:
            dx = 0.0
            dz = foot_length
            horizontal_length = foot_length

        max_vertical_offset = min(0.015, foot_length * 0.10)
        dy = max(-max_vertical_offset, min(max_vertical_offset, foot_position[1] - ankle_position[1]))
        horizontal_target = math.sqrt(max(0.0, (foot_length * foot_length) - (dy * dy)))
        scale = horizontal_target / horizontal_length
        dz = abs(dz * scale)
        dx = dx * scale

        corrected_position = (
            ankle_position[0] + dx,
            ankle_position[1] + dy,
            ankle_position[2] + dz,
        )
        if joint_distance(foot_position, corrected_position) > 1e-9 and diagnostics is not None:
            diagnostics.foot_forward_orientation_fixes += 1
        positions[foot_id] = corrected_position


def reconstruct_positions_from_quaternions(
    quaternions: dict[str, dict[str, float]],
    lengths: dict[tuple[int, int], float],
    diagnostics: ReconstructionDiagnostics | None = None,
) -> dict[int, tuple[float, float, float]]:
    # Grow the skeleton outward from known segments by rotating each parent limb
    # into its child direction and then applying the configured bone length.
    positions = seed_root_limb_positions(quaternions, lengths)

    changed = True
    while changed:
        changed = False
        for joint_a, joint_b, joint_c in CONNECTED_LIMB_CHAINS:
            column = f"{joint_a}-{joint_b}-{joint_c}"
            quaternion = compose_with_t_pose_reference(column, quaternions[column])

            if joint_a in positions and joint_b in positions and joint_c not in positions:
                parent_vector = subtract_position_vectors(positions[joint_b], positions[joint_a])
                child_direction = rotate_vector(parent_vector, quaternion)
                positions[joint_c] = add_vectors(
                    positions[joint_b],
                    scale_vector(child_direction, limb_length_for(lengths, joint_b, joint_c)),
                )
                changed = True

    fill_remaining_default_positions(positions, lengths)
    enforce_leg_side_separation(positions, diagnostics)
    enforce_leg_downward_orientation(positions, diagnostics)
    enforce_foot_forward_orientation(positions, lengths, diagnostics)
    return positions


def reconstruct_frames_from_csv(
    pose_csv_path: Path,
    limb_lengths_csv_path: Path,
    joint_limits_path: Path | None = None,
    diagnostics: ReconstructionDiagnostics | None = None,
) -> list[dict]:
    # Visualization/evaluation use this inverse path: normalized pose CSV +
    # shared limb lengths -> reconstructed joint positions per frame.
    lengths = load_limb_lengths_csv(limb_lengths_csv_path)
    pose_frames = load_pose_csv(pose_csv_path, joint_limits_path)
    reconstructed_frames = [
        {
            "frame_index": frame["frame_index"],
            "joints": reconstruct_positions_from_quaternions(frame["limb_angle_quaternions"], lengths, diagnostics),
        }
        for frame in pose_frames
    ]
    # TODO: Add explicit jolt diagnostics here before reintroducing any smoothing.
    return reconstructed_frames


def build_processed_path(recording_path: Path, processed_directory: Path) -> Path:
    processed_directory.mkdir(parents=True, exist_ok=True)
    sample_id = recording_path.stem.removeprefix("recording_")
    return processed_directory / f"processed_{sample_id}.csv"


def build_limb_lengths_path(data_directory: Path) -> Path:
    data_directory.mkdir(parents=True, exist_ok=True)
    return data_directory / "limb_lengths.csv"


def resolve_existing_limb_lengths_path(recording_path: Path, processed_directory: Path) -> Path:
    data_directory = processed_directory.parent.parent
    preferred_path = build_limb_lengths_path(data_directory)
    if preferred_path.exists():
        return preferred_path

    sample_id = recording_path.stem.removeprefix("recording_")
    processed_sidecar_path = processed_directory / f"limb_lengths_{sample_id}.csv"
    if processed_sidecar_path.exists():
        return processed_sidecar_path

    legacy_path = processed_directory.parent.parent / f"limb_lengths_{recording_path.stem.removeprefix('recording_')}.csv"
    if legacy_path.exists():
        return legacy_path

    return preferred_path


def save_processed_csv(
    processed_frames: list[dict],
    output_path: Path,
    joint_limits_path: Path,
) -> None:
    # Persist the processed pose as normalized 0..1 rotation-vector components
    # per connected chain. Reconstruction converts these values back to
    # quaternions through the same joint limit table.
    if output_path.exists():
        output_path.unlink()

    chain_limits = load_joint_limits(joint_limits_path)
    limb_columns = [
        *ROOT_LIMB_COLUMNS,
        *[f"{joint_a}-{joint_b}-{joint_c}" for joint_a, joint_b, joint_c in CONNECTED_LIMB_CHAINS],
    ]
    fieldnames = ["frame_index", *limb_columns]

    rows: list[dict[str, str]] = []
    for frame in processed_frames:
        quaternions = frame["limb_angle_quaternions"]
        row = {"frame_index": str(frame["frame_index"])}
        for column in limb_columns:
            limit = chain_limits.get(column)
            if limit is None:
                raise ValueError(f"Pose column {column!r} has no entry in joint_limits.csv")

            row[column] = format_normalized_limit_values(
                quaternion_to_normalized_limit_values(quaternions[column], limit)
            )
        rows.append(row)

    rows = filter_normalized_pose_jitter(rows)

    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def print_recording_summary(frames: list[dict], processed_path: Path) -> None:
    print(f"Loaded {len(frames)} raw frames")
    if frames:
        print(
            f"First frame delta: {frames[0]['delta_time']} ticks, "
            f"tracked joints: {len(frames[0]['joints'])}"
        )
    print(f"Wrote processed output to {processed_path}")


def main() -> int:
    code_directory = Path(__file__).resolve().parent.parent
    data_directory = code_directory / "data"
    raw_directory = data_directory / "recordings" / "raw"
    processed_directory = data_directory / "recordings" / "processed"

    parser = argparse.ArgumentParser(description="Read raw EMG pose recordings and write processed features.")
    parser.add_argument(
        "recording",
        nargs="?",
        default="0",
        help="Raw recording path or numeric sample id like 0, 1, 2",
    )
    parser.add_argument("--joint-limits", default=str(data_directory / "joint_limits.csv"))
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
    centered_frames = filter_joint_jitter(centered_frames)
    processed_frames = preprocess_frames(centered_frames)
    if not processed_frames:
        raise ValueError(
            f"No usable frames found in {recording_path}. "
            "The recording does not contain all required non-foot joints after resampling."
        )

    processed_path = build_processed_path(recording_path, processed_directory)
    save_processed_csv(processed_frames, processed_path, Path(args.joint_limits))
    print_recording_summary(filled_frames, processed_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

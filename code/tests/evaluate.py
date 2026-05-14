import argparse
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from process_data.preprocess import (
    ReconstructionDiagnostics,
    ROOT_JOINT_ID,
    SKELETON_BONES,
    interpolate_missing_joint_positions,
    load_recording,
    normalize_root_visibility,
    reconstruct_frames_from_csv,
    resolve_existing_limb_info_path,
    resample_frames,
)


def load_raw_ground_truth_frames(path: Path) -> list[dict]:
    raw_frames = load_recording(path)
    resampled_frames = resample_frames(raw_frames)
    normalized_frames = normalize_root_visibility(resampled_frames)
    filled_frames = interpolate_missing_joint_positions(normalized_frames)

    frames: list[dict] = []
    for index, frame in enumerate(filled_frames):
        joints = frame["joints"]
        root = joints.get(ROOT_JOINT_ID, (0.0, 0.0, 0.0))
        centered_joints = {
            joint_id: (
                position[0] - root[0],
                position[1] - root[1],
                position[2] - root[2],
            )
            for joint_id, position in joints.items()
        }
        frames.append({"frame_index": index, "joints": centered_joints})

    return frames


def distance(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
    dx = left[0] - right[0]
    dy = left[1] - right[1]
    dz = left[2] - right[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def evaluate_pose(ground_truth_frames: list[dict], prediction_frames: list[dict]) -> dict[str, float]:
    frame_count = min(len(ground_truth_frames), len(prediction_frames))
    joint_errors: list[float] = []
    squared_joint_errors: list[float] = []
    bone_length_errors: list[float] = []

    for frame_index in range(frame_count):
        ground_truth_joints = ground_truth_frames[frame_index]["joints"]
        prediction_joints = prediction_frames[frame_index]["joints"]
        common_joints = sorted(set(ground_truth_joints) & set(prediction_joints))

        for joint_id in common_joints:
            error = distance(ground_truth_joints[joint_id], prediction_joints[joint_id])
            joint_errors.append(error)
            squared_joint_errors.append(error * error)

        for joint_a, joint_b in SKELETON_BONES:
            if joint_a not in prediction_joints or joint_b not in prediction_joints:
                continue
            if joint_a not in ground_truth_joints or joint_b not in ground_truth_joints:
                continue

            prediction_length = distance(prediction_joints[joint_a], prediction_joints[joint_b])
            ground_truth_length = distance(ground_truth_joints[joint_a], ground_truth_joints[joint_b])
            bone_length_errors.append(abs(prediction_length - ground_truth_length))

    joint_count = len(joint_errors)
    bone_count = len(bone_length_errors)

    return {
        "frames": frame_count,
        "joint_samples": joint_count,
        "joint_mae_m": sum(joint_errors) / joint_count if joint_count else 0.0,
        "joint_rmse_m": math.sqrt(sum(squared_joint_errors) / joint_count) if joint_count else 0.0,
        "bone_length_mae_m": sum(bone_length_errors) / bone_count if bone_count else 0.0,
    }


def main() -> int:
    code_directory = Path(__file__).resolve().parent.parent
    data_directory = code_directory / "data"
    processed_directory = data_directory / "recordings" / "processed"

    parser = argparse.ArgumentParser(description="Evaluate predicted normalized pose against raw skeletal ground truth.")
    parser.add_argument("sample_id", nargs="?", default="0", help="Numeric sample id, e.g. 0 or 1.")
    args = parser.parse_args()

    raw_path = data_directory / "recordings" / "raw" / f"recording_{args.sample_id}.bin"
    prediction_path = data_directory / "predictions" / f"prediction_{args.sample_id}.csv"
    limb_info_path = resolve_existing_limb_info_path(raw_path, processed_directory)

    ground_truth_frames = load_raw_ground_truth_frames(raw_path)
    reconstruction_diagnostics = ReconstructionDiagnostics()
    prediction_frames = reconstruct_frames_from_csv(
        prediction_path,
        limb_info_path,
        diagnostics=reconstruction_diagnostics,
    )
    metrics = evaluate_pose(ground_truth_frames, prediction_frames)

    for key, value in metrics.items():
        print(f"{key}: {value}")
    print(f"leg_side_separation_fixes: {reconstruction_diagnostics.leg_side_separation_fixes}")
    print(f"leg_downward_orientation_fixes: {reconstruction_diagnostics.leg_downward_orientation_fixes}")
    print(f"foot_forward_orientation_fixes: {reconstruction_diagnostics.foot_forward_orientation_fixes}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

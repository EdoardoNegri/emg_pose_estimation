from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from process_data.preprocess import (
    T_POSE_CENTERED_JOINTS,
    calculate_limb_angle_quaternions,
    joint_dict_from_tuple,
    load_joint_limits,
    load_limb_lengths_csv,
    quaternion_to_normalized_limit_values,
    quaternion_to_rotation_vector_degrees,
    reconstruct_positions_from_quaternions,
)


ZERO_TOLERANCE_DEGREES = 1e-9
NORMALIZED_TOLERANCE = 1e-9
SYMMETRY_TOLERANCE_M = 1e-6


def assert_close(value: float, expected: float, tolerance: float, message: str) -> None:
    if abs(value - expected) > tolerance:
        raise AssertionError(f"{message}: got {value}, expected {expected}")


def expected_zero_normalized(minimum: float, maximum: float) -> float:
    if abs(maximum - minimum) <= 1e-12:
        return 0.5
    return (0.0 - minimum) / (maximum - minimum)


def main() -> int:
    code_directory = Path(__file__).resolve().parent.parent
    centered_joints = {
        str(joint_id): joint_dict_from_tuple(position)
        for joint_id, position in T_POSE_CENTERED_JOINTS.items()
    }
    quaternions = calculate_limb_angle_quaternions(centered_joints)
    chain_limits = load_joint_limits(code_directory / "data" / "joint_limits.csv")

    for chain, quaternion in quaternions.items():
        rotation_vector = quaternion_to_rotation_vector_degrees(quaternion)
        for component in rotation_vector:
            assert_close(
                component,
                0.0,
                ZERO_TOLERANCE_DEGREES,
                f"{chain} is not zero in the T-pose reference",
            )

        normalized_values = quaternion_to_normalized_limit_values(quaternion, chain_limits[chain])
        expected_values = (
            expected_zero_normalized(chain_limits[chain].min_x, chain_limits[chain].max_x),
            expected_zero_normalized(chain_limits[chain].min_y, chain_limits[chain].max_y),
            expected_zero_normalized(chain_limits[chain].min_z, chain_limits[chain].max_z),
        )
        for actual, expected in zip(normalized_values, expected_values):
            assert_close(
                actual,
                expected,
                NORMALIZED_TOLERANCE,
                f"{chain} normalized T-pose value does not match joint_limits lerp",
            )

    limb_lengths = load_limb_lengths_csv(code_directory / "data" / "limb_lengths.csv")
    joints = reconstruct_positions_from_quaternions(quaternions, limb_lengths)

    for ankle_id, foot_id in ((14, 15), (18, 19)):
        assert_close(
            joints[foot_id][1],
            joints[ankle_id][1],
            SYMMETRY_TOLERANCE_M,
            f"foot {foot_id} should stay level with ankle {ankle_id}",
        )
        if joints[foot_id][2] <= joints[ankle_id][2]:
            raise AssertionError(f"foot {foot_id} should point forward in +Z")

    for hand_tip_id, thumb_id in ((21, 22), (23, 24)):
        if joints[thumb_id][2] <= joints[hand_tip_id][2]:
            raise AssertionError(f"thumb {thumb_id} should point forward in +Z")

    for left_id, right_id in ((12, 16), (13, 17), (14, 18), (15, 19), (21, 23), (22, 24)):
        assert_close(joints[left_id][0], -joints[right_id][0], SYMMETRY_TOLERANCE_M, f"{left_id}/{right_id} X symmetry")
        assert_close(joints[left_id][1], joints[right_id][1], SYMMETRY_TOLERANCE_M, f"{left_id}/{right_id} Y symmetry")
        assert_close(joints[left_id][2], joints[right_id][2], SYMMETRY_TOLERANCE_M, f"{left_id}/{right_id} Z symmetry")

    print("zero pose checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

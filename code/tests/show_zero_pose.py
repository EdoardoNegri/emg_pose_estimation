import argparse
import csv
import math
from pathlib import Path
import sys
import tkinter as tk

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from process_data.preprocess import (
    CONNECTED_LIMB_CHAINS,
    ROOT_LIMB_COLUMNS,
    T_POSE_CENTERED_JOINTS,
    calculate_limb_angle_quaternions,
    format_normalized_limit_values,
    joint_dict_from_tuple,
    load_joint_limits,
    load_limb_lengths_csv,
    quaternion_to_normalized_limit_values,
    quaternion_to_rotation_vector_degrees,
    reconstruct_positions_from_quaternions,
)
from tests.visualize import (
    SKELETON_BONES,
    color_for_joint,
)


WINDOW_WIDTH = 960
WINDOW_HEIGHT = 720
JOINT_RADIUS = 4


def chain_columns() -> list[str]:
    return [
        *ROOT_LIMB_COLUMNS,
        *[f"{joint_a}-{joint_b}-{joint_c}" for joint_a, joint_b, joint_c in CONNECTED_LIMB_CHAINS],
    ]


def expected_zero_normalized(minimum: float, maximum: float) -> float:
    if abs(maximum - minimum) <= 1e-12:
        return 0.5
    return (0.0 - minimum) / (maximum - minimum)


def project_front(joint: tuple[float, float, float], scale: float) -> tuple[float, float]:
    x, y, _ = joint
    return WINDOW_WIDTH * 0.5 + x * scale, WINDOW_HEIGHT * 0.50 - y * scale


def rotate_joint(joint: tuple[float, float, float], yaw: float, pitch: float) -> tuple[float, float, float]:
    x, y, z = joint

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    xz_x = (x * cos_yaw) + (z * sin_yaw)
    xz_z = (-x * sin_yaw) + (z * cos_yaw)

    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)
    rotated_y = (y * cos_pitch) - (xz_z * sin_pitch)
    rotated_z = (y * sin_pitch) + (xz_z * cos_pitch)

    return xz_x, rotated_y, rotated_z


def draw_zero_pose(joints: dict[int, tuple[float, float, float]]) -> None:
    root = tk.Tk()
    root.title("Zero Pose: T-pose with palms down")

    canvas = tk.Canvas(root, width=WINDOW_WIDTH, height=WINDOW_HEIGHT, bg="white")
    canvas.pack(fill="both", expand=True)

    x_extent = max((max(abs(position[0]), abs(position[2])) for position in joints.values()), default=0.5)
    y_extent = max((abs(position[1]) for position in joints.values()), default=0.5)
    scale = min((WINDOW_WIDTH * 0.38) / max(x_extent, 0.5), (WINDOW_HEIGHT * 0.42) / max(y_extent, 0.5))

    yaw = 0.0
    pitch = 0.0
    rotation_step = math.radians(8.0)

    def draw() -> None:
        canvas.delete("all")
        rotated_joints = {
            joint_id: rotate_joint(position, yaw, pitch)
            for joint_id, position in joints.items()
        }

        projected = {
            joint_id: project_front(position, scale)
            for joint_id, position in rotated_joints.items()
        }

        for joint_a, joint_b in SKELETON_BONES:
            if joint_a not in projected or joint_b not in projected:
                continue
            x0, y0 = projected[joint_a]
            x1, y1 = projected[joint_b]
            canvas.create_line(x0, y0, x1, y1, fill="#00a7ff", width=4)

        for joint_id, (x, y) in projected.items():
            color = color_for_joint(joint_id)
            canvas.create_oval(
                x - JOINT_RADIUS,
                y - JOINT_RADIUS,
                x + JOINT_RADIUS,
                y + JOINT_RADIUS,
                fill=color,
                outline="",
            )
            canvas.create_text(x + 8, y - 8, text=str(joint_id), fill=color, anchor="w", font=("Segoe UI", 8))

        canvas.create_text(
            24,
            24,
            text="Zero pose: internal rotation-vector (0, 0, 0) for every limb",
            fill="#222222",
            anchor="w",
            font=("Segoe UI", 12, "bold"),
        )
        canvas.create_text(
            24,
            48,
            text="Arrow keys rotate view. Feet point forward on Z.",
            fill="#444444",
            anchor="w",
            font=("Segoe UI", 10),
        )
        canvas.create_text(
            WINDOW_WIDTH - 24,
            24,
            text=f"yaw={math.degrees(yaw):.0f}  pitch={math.degrees(pitch):.0f}",
            fill="#444444",
            anchor="e",
            font=("Segoe UI", 10),
        )

    def handle_key(event: tk.Event) -> None:
        nonlocal yaw, pitch
        if event.keysym == "Left":
            yaw -= rotation_step
        elif event.keysym == "Right":
            yaw += rotation_step
        elif event.keysym == "Up":
            pitch += rotation_step
        elif event.keysym == "Down":
            pitch -= rotation_step
        else:
            return
        draw()

    root.bind("<Left>", handle_key)
    root.bind("<Right>", handle_key)
    root.bind("<Up>", handle_key)
    root.bind("<Down>", handle_key)
    root.focus_set()
    draw()

    root.mainloop()


def main() -> int:
    code_directory = Path(__file__).resolve().parent.parent
    data_directory = code_directory / "data"

    parser = argparse.ArgumentParser(description="Print and save the T-pose zero-rotation reference.")
    parser.add_argument("--limb-info", default=str(data_directory / "limb_info.csv"))
    parser.add_argument("--output", default=str(data_directory / "tests" / "zero_pose.csv"))
    parser.add_argument("--no-window", action="store_true", help="Only print/write the zero pose; do not open Tkinter.")
    args = parser.parse_args()

    centered_joints = {
        str(joint_id): joint_dict_from_tuple(position)
        for joint_id, position in T_POSE_CENTERED_JOINTS.items()
    }
    quaternions = calculate_limb_angle_quaternions(centered_joints)
    limb_info_path = Path(args.limb_info)
    chain_limits = load_joint_limits(limb_info_path)
    limb_lengths = load_limb_lengths_csv(limb_info_path)

    max_zero_error = 0.0
    print("Internal rotation-vector degrees for T-pose with palms down:")
    for column in chain_columns():
        rotation_vector = quaternion_to_rotation_vector_degrees(quaternions[column])
        max_zero_error = max(max_zero_error, *(abs(component) for component in rotation_vector))
        print(f"{column}: {rotation_vector[0]:.6f} {rotation_vector[1]:.6f} {rotation_vector[2]:.6f}")

    print()
    print(f"max absolute zero error: {max_zero_error:.12f} degrees")

    print()
    print("Normalized T-pose values from limb_info.csv:")
    for column in chain_columns():
        limit = chain_limits[column]
        expected_values = (
            expected_zero_normalized(limit.min_x, limit.max_x),
            expected_zero_normalized(limit.min_y, limit.max_y),
            expected_zero_normalized(limit.min_z, limit.max_z),
        )
        actual_values = quaternion_to_normalized_limit_values(quaternions[column], limit)
        print(
            f"{column}: "
            f"expected={expected_values[0]:.9f} {expected_values[1]:.9f} {expected_values[2]:.9f}  "
            f"actual={actual_values[0]:.9f} {actual_values[1]:.9f} {actual_values[2]:.9f}"
        )

    reconstructed = reconstruct_positions_from_quaternions(quaternions, limb_lengths)
    print()
    print("Reconstructed zero-pose joints:")
    for joint_id in sorted(reconstructed):
        x, y, z = reconstructed[joint_id]
        print(f"{joint_id}: {x:.6f} {y:.6f} {z:.6f}")

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    row = {"frame_index": "0"}
    for column in chain_columns():
        row[column] = format_normalized_limit_values(
            quaternion_to_normalized_limit_values(quaternions[column], chain_limits[column])
        )

    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["frame_index", *chain_columns()])
        writer.writeheader()
        writer.writerow(row)

    print()
    print(f"wrote normalized zero-pose row to {output_path}")

    if not args.no_window:
        try:
            draw_zero_pose(reconstructed)
        except tk.TclError as error:
            print(f"Could not open Tkinter window: {error}")
            return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

import argparse
from pathlib import Path
import tkinter as tk

from preprocess import (
    ROOT_JOINT_ID,
    interpolate_missing_joint_positions,
    load_recording,
    normalize_root_visibility,
    reconstruct_frames_from_csv,
    resample_frames,
)


WINDOW_WIDTH = 960
WINDOW_HEIGHT = 720
JOINT_RADIUS = 4
PLAYBACK_FPS = 30
PLAYBACK_DELAY_MS = round(1000 / PLAYBACK_FPS)
GROUND_TRUTH_BONE_COLOR = "#111111"
PREDICTION_BONE_COLOR = "#00a7ff"
ALL_JOINT_IDS = tuple(range(25))
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
JOINT_PALETTE = (
    "#e6194b",
    "#3cb44b",
    "#ffe119",
    "#4363d8",
    "#f58231",
    "#911eb4",
    "#46f0f0",
    "#f032e6",
    "#bcf60c",
    "#fabebe",
    "#008080",
    "#e6beff",
    "#9a6324",
    "#fffac8",
    "#800000",
    "#aaffc3",
    "#808000",
    "#ffd8b1",
    "#000075",
    "#808080",
    "#ff6f61",
    "#6b5b95",
    "#88b04b",
    "#f7cac9",
    "#92a8d1",
)


def require_path(path: Path) -> Path:
    if not path.exists():
        raise FileNotFoundError(f"Required file not found: {path}")
    return path


def resolve_paths(script_directory: Path, sample_id: int) -> tuple[Path, Path, Path]:
    data_directory = script_directory / "data"
    raw_path = data_directory / "recordings" / "raw" / f"recording_{sample_id}.bin"
    prediction_path = data_directory / "predictions" / f"prediction_{sample_id}.csv"
    limb_lengths_path = data_directory / f"limb_lengths_{sample_id}.csv"
    return raw_path, prediction_path, limb_lengths_path


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


def project_joint(joint: tuple[float, float, float], scale: float) -> tuple[float, float]:
    x, y, _ = joint
    screen_x = WINDOW_WIDTH / 2 + x * scale
    screen_y = WINDOW_HEIGHT * 0.85 - y * scale
    return screen_x, screen_y


def color_for_joint(joint_id: int) -> str:
    return JOINT_PALETTE[joint_id % len(JOINT_PALETTE)]


def draw_skeleton(
    canvas: tk.Canvas,
    joints: dict[int, tuple[float, float, float]],
    scale: float,
    bone_color: str,
) -> None:
    projected = {
        joint_id: project_joint(position, scale)
        for joint_id, position in joints.items()
    }

    for joint0, joint1 in SKELETON_BONES:
        if joint0 not in projected or joint1 not in projected:
            continue
        x0, y0 = projected[joint0]
        x1, y1 = projected[joint1]
        canvas.create_line(x0, y0, x1, y1, fill=bone_color, width=3)

    for joint_id, (x, y) in projected.items():
        joint_color = color_for_joint(joint_id)
        canvas.create_oval(
            x - JOINT_RADIUS,
            y - JOINT_RADIUS,
            x + JOINT_RADIUS,
            y + JOINT_RADIUS,
            fill=joint_color,
            outline="",
        )


def draw_prediction_legend(canvas: tk.Canvas) -> None:
    legend_x = 28
    legend_y = 110

    canvas.create_text(legend_x, 28, text="Legend", fill="#222222", anchor="w", font=("Segoe UI", 12, "bold"))
    canvas.create_line(legend_x, 50, legend_x + 28, 50, fill=GROUND_TRUTH_BONE_COLOR, width=3)
    canvas.create_text(legend_x + 40, 50, text="Ground truth bones", fill="#2f6f3e", anchor="w", font=("Segoe UI", 9, "bold"))
    canvas.create_line(legend_x, 74, legend_x + 28, 74, fill=PREDICTION_BONE_COLOR, width=3)
    canvas.create_text(legend_x + 40, 74, text="Prediction bones", fill="#355cde", anchor="w", font=("Segoe UI", 9, "bold"))

    for row_index, joint_id in enumerate(ALL_JOINT_IDS):
        column = row_index // 13
        row = row_index % 13
        x = legend_x + column * 110
        y = legend_y + row * 18
        joint_color = color_for_joint(joint_id)
        canvas.create_oval(
            x,
            y - 6,
            x + 12,
            y + 6,
            fill=joint_color,
            outline="",
        )
        canvas.create_text(x + 22, y, text=str(joint_id), fill=joint_color, anchor="w", font=("Segoe UI", 8))


def replay_processed(ground_truth_frames: list[dict], prediction_frames: list[dict]) -> None:
    if not ground_truth_frames:
        print("No frames to replay.")
        return

    root = tk.Tk()
    root.title("Prediction Comparison")

    canvas = tk.Canvas(root, width=WINDOW_WIDTH, height=WINDOW_HEIGHT, bg="white")
    canvas.pack(fill="both", expand=True)

    all_x: list[float] = []
    all_y: list[float] = []
    for frame in ground_truth_frames + prediction_frames:
        for x, y, _ in frame["joints"].values():
            all_x.append(x)
            all_y.append(y)

    x_extent = max((max(abs(value) for value in all_x), 0.5), default=0.5)
    y_min = min(all_y, default=-1.0)
    y_max = max(all_y, default=1.0)
    y_extent = max(abs(y_min), abs(y_max), 0.5)
    scale = min((WINDOW_WIDTH * 0.35) / x_extent, (WINDOW_HEIGHT * 0.40) / y_extent)

    status_var = tk.StringVar(value="Frame 0")
    status_label = tk.Label(root, textvariable=status_var, anchor="w")
    status_label.pack(fill="x")

    is_paused = False
    current_frame_index = 0
    scheduled_job = None

    def cancel_scheduled_job() -> None:
        nonlocal scheduled_job
        if scheduled_job is not None:
            root.after_cancel(scheduled_job)
            scheduled_job = None

    def draw_frame(frame_index: int) -> None:
        nonlocal current_frame_index, scheduled_job
        current_frame_index = frame_index
        ground_truth_frame = ground_truth_frames[frame_index]

        canvas.delete("all")
        draw_skeleton(canvas, ground_truth_frame["joints"], scale, bone_color=GROUND_TRUTH_BONE_COLOR)
        draw_prediction_legend(canvas)

        if frame_index < len(prediction_frames):
            draw_skeleton(canvas, prediction_frames[frame_index]["joints"], scale, bone_color=PREDICTION_BONE_COLOR)

        status_var.set(
            f"Frame {frame_index + 1}/{len(ground_truth_frames)}  "
            f"fps={PLAYBACK_FPS}  "
            f"paused={'yes' if is_paused else 'no'}"
        )

        next_index = frame_index + 1
        if is_paused or next_index >= len(ground_truth_frames):
            return

        scheduled_job = root.after(PLAYBACK_DELAY_MS, lambda: draw_frame(next_index))

    def step_frame(step: int) -> None:
        cancel_scheduled_job()
        next_index = max(0, min(len(ground_truth_frames) - 1, current_frame_index + step))
        draw_frame(next_index)

    def toggle_pause(_event=None) -> None:
        nonlocal is_paused
        is_paused = not is_paused
        cancel_scheduled_job()
        draw_frame(current_frame_index)

    def go_previous(_event=None) -> None:
        nonlocal is_paused
        is_paused = True
        step_frame(-1)

    def go_next(_event=None) -> None:
        nonlocal is_paused
        is_paused = True
        step_frame(1)

    root.bind("<space>", toggle_pause)
    root.bind("<Left>", go_previous)
    root.bind("<Right>", go_next)

    draw_frame(0)
    root.mainloop()


def main() -> int:
    script_directory = Path(__file__).resolve().parent

    parser = argparse.ArgumentParser(description="Visualize ground truth and prediction overlays by sample id.")
    parser.add_argument("sample_id", type=int, help="Numeric sample id, e.g. 0 or 1.")
    args = parser.parse_args()

    raw_path, prediction_path, limb_lengths_path = resolve_paths(script_directory, args.sample_id)
    try:
        ground_truth_frames = load_raw_ground_truth_frames(require_path(raw_path))
        prediction_frames = reconstruct_frames_from_csv(require_path(prediction_path), require_path(limb_lengths_path))
    except FileNotFoundError as error:
        print(error)
        print(f"Run these first:")
        print(f"  python preprocess.py {args.sample_id}")
        print(f"  python model.py {args.sample_id}")
        return 1

    try:
        replay_processed(ground_truth_frames, prediction_frames)
    except tk.TclError as error:
        print(f"Could not open Tkinter window: {error}")
        print("Your Python Tk/Tcl install looks incomplete. Reinstall Python with Tcl/Tk support or run with a Python that has tkinter working.")
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

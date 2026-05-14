import argparse
from pathlib import Path
import sys
import time
import tkinter as tk

# This viewer overlays ground-truth skeleton motion against reconstructed
# prediction motion using the shared limb info metadata used by evaluation.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from process_data.preprocess import (
    ROOT_JOINT_ID,
    SKELETON_BONES,
    interpolate_missing_joint_positions,
    load_recording,
    normalize_root_visibility,
    reconstruct_frames_from_csv,
    resample_frames,
    resolve_existing_limb_info_path,
)


WINDOW_WIDTH = 960
WINDOW_HEIGHT = 720
JOINT_RADIUS = 4
SOURCE_FPS = 60
RENDER_FPS = 60
SOURCE_FRAME_DURATION_S = 1.0 / SOURCE_FPS
RENDER_FRAME_DURATION_S = 1.0 / RENDER_FPS
GROUND_TRUTH_BONE_COLOR = "#111111"
PREDICTION_BONE_COLOR = "#00a7ff"
ALL_JOINT_IDS = tuple(range(25))
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


def resolve_paths(code_directory: Path, sample_id: int) -> tuple[Path, Path, Path]:
    data_directory = code_directory / "data"
    processed_directory = data_directory / "recordings" / "processed"
    raw_path = data_directory / "recordings" / "raw" / f"recording_{sample_id}.bin"
    prediction_path = data_directory / "predictions" / f"prediction_{sample_id}.csv"
    limb_info_path = resolve_existing_limb_info_path(raw_path, processed_directory)
    return raw_path, prediction_path, limb_info_path


def load_raw_ground_truth_frames(path: Path) -> list[dict]:
    # Reuse the preprocessing path so the ground-truth clip shown in the viewer
    # matches the timing/visibility assumptions used by prediction playback.
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
    # The viewer is intentionally a simple orthographic front projection.
    x, y, _ = joint
    screen_x = WINDOW_WIDTH / 2 + x * scale
    screen_y = WINDOW_HEIGHT * 0.42 - y * scale
    return screen_x, screen_y


def color_for_joint(joint_id: int) -> str:
    return JOINT_PALETTE[joint_id % len(JOINT_PALETTE)]


def draw_skeleton(canvas: tk.Canvas, joints: dict[int, tuple[float, float, float]], scale: float, bone_color: str) -> None:
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
    # Keep the legend drawn in-canvas so screenshots remain self-explanatory.
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
    # Tkinter is used here as a lightweight debug viewer, not a high-fidelity
    # renderer. The playback loop is driven by wall-clock time.
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
    playback_started_at: float | None = None

    def cancel_scheduled_job() -> None:
        nonlocal scheduled_job
        if scheduled_job is not None:
            root.after_cancel(scheduled_job)
            scheduled_job = None

    def draw_frame(frame_index: int) -> None:
        nonlocal current_frame_index, scheduled_job, playback_started_at
        current_frame_index = max(0, min(frame_index, len(ground_truth_frames) - 1))
        ground_truth_joints = ground_truth_frames[current_frame_index]["joints"]

        canvas.delete("all")
        draw_skeleton(canvas, ground_truth_joints, scale, bone_color=GROUND_TRUTH_BONE_COLOR)
        draw_prediction_legend(canvas)

        if current_frame_index < len(prediction_frames):
            draw_skeleton(canvas, prediction_frames[current_frame_index]["joints"], scale, bone_color=PREDICTION_BONE_COLOR)

        status_var.set(
            f"Frame {current_frame_index + 1}/{len(ground_truth_frames)}  "
            f"fps={SOURCE_FPS}  "
            f"paused={'yes' if is_paused else 'no'}"
        )

        if is_paused:
            return

        if playback_started_at is None:
            playback_started_at = time.perf_counter() - (current_frame_index * SOURCE_FRAME_DURATION_S)

        # Loop playback by mapping elapsed wall-clock time back onto the frame
        # range, instead of incrementing frames blindly.
        loop_duration_s = len(ground_truth_frames) * SOURCE_FRAME_DURATION_S
        now = time.perf_counter()
        elapsed_s = now - playback_started_at
        if loop_duration_s > 0.0:
            elapsed_s = elapsed_s % loop_duration_s

        real_next_index = int(elapsed_s * SOURCE_FPS) % len(ground_truth_frames)
        target_time = playback_started_at + (real_next_index * SOURCE_FRAME_DURATION_S)
        if target_time <= now:
            target_time += SOURCE_FRAME_DURATION_S
        delay_ms = max(1, round((target_time - time.perf_counter()) * 1000))
        scheduled_job = root.after(delay_ms, lambda: draw_frame(real_next_index))

    def step_frame(step: int) -> None:
        cancel_scheduled_job()
        next_index = max(0, min(len(ground_truth_frames) - 1, current_frame_index + step))
        draw_frame(next_index)

    def toggle_pause(_event=None) -> None:
        nonlocal is_paused, playback_started_at
        is_paused = not is_paused
        cancel_scheduled_job()
        if not is_paused:
            playback_started_at = time.perf_counter() - (current_frame_index * SOURCE_FRAME_DURATION_S)
        draw_frame(current_frame_index)

    def go_previous(_event=None) -> None:
        nonlocal is_paused, playback_started_at
        is_paused = True
        playback_started_at = None
        step_frame(-1)

    def go_next(_event=None) -> None:
        nonlocal is_paused, playback_started_at
        is_paused = True
        playback_started_at = None
        step_frame(1)

    root.bind("<space>", toggle_pause)
    root.bind("<Left>", go_previous)
    root.bind("<Right>", go_next)

    draw_frame(0)
    root.mainloop()


def main() -> int:
    code_directory = Path(__file__).resolve().parent.parent

    parser = argparse.ArgumentParser(description="Visualize ground truth and prediction overlays by sample id.")
    parser.add_argument("sample_id", type=int, help="Numeric sample id, e.g. 0 or 1.")
    args = parser.parse_args()

    raw_path, prediction_path, limb_info_path = resolve_paths(code_directory, args.sample_id)
    try:
        ground_truth_frames = load_raw_ground_truth_frames(require_path(raw_path))
        prediction_frames = reconstruct_frames_from_csv(require_path(prediction_path), require_path(limb_info_path))
    except FileNotFoundError as error:
        print(error)
        print(f"Run these first:")
        print(f"  python process_data\\preprocess.py {args.sample_id}")
        print(f"  python process_data\\model.py {args.sample_id}")
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

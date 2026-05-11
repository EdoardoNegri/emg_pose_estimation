from __future__ import annotations

import argparse
import csv
import json
import math
import random
from pathlib import Path

# This module currently serves two roles:
# 1) create a baseline prediction by perturbing/clamping processed quaternions
# 2) provide an optional PyTorch sequence model for later training experiments
try:
    import torch
    from torch import nn
    from torch.utils.data import DataLoader, Dataset
except ModuleNotFoundError:
    torch = None
    nn = None
    DataLoader = None

    class Dataset:
        pass


JOINT_IDS = tuple(range(25))
FEATURES_PER_JOINT = 3
QUATERNION_COMPONENTS = ("w", "x", "y", "z")
DEFAULT_MAX_ANGLE_DEGREES = 160.0


def parse_quaternion(value: str) -> dict[str, float]:
    components = [float(component) for component in value.split()]
    if len(components) != len(QUATERNION_COMPONENTS):
        raise ValueError(f"Expected 4 quaternion components, got {len(components)} in {value!r}")

    return dict(zip(QUATERNION_COMPONENTS, components))


def format_quaternion(quaternion: dict[str, float]) -> str:
    return " ".join(str(quaternion[component]) for component in QUATERNION_COMPONENTS)


def normalize_quaternion(quaternion: dict[str, float]) -> dict[str, float]:
    length = math.sqrt(sum(quaternion[component] * quaternion[component] for component in QUATERNION_COMPONENTS))
    if length <= 1e-12:
        return {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}

    return {
        component: quaternion[component] / length
        for component in QUATERNION_COMPONENTS
    }


def add_quaternion_noise(value: str, noise_std: float, random_generator: random.Random) -> str:
    quaternion = parse_quaternion(value)
    noisy_quaternion = {
        component: quaternion[component] + random_generator.gauss(0.0, noise_std)
        for component in QUATERNION_COMPONENTS
    }
    return format_quaternion(normalize_quaternion(noisy_quaternion))


def clamp_quaternion_angle(quaternion: dict[str, float], max_angle_degrees: float) -> dict[str, float]:
    normalized = normalize_quaternion(quaternion)
    w = max(-1.0, min(1.0, normalized["w"]))
    angle = 2.0 * math.acos(w)
    max_angle = math.radians(max_angle_degrees)

    if angle <= max_angle:
        return normalized

    sin_half_angle = math.sqrt(max(0.0, 1.0 - w * w))
    if sin_half_angle <= 1e-12:
        return normalized

    axis = (
        normalized["x"] / sin_half_angle,
        normalized["y"] / sin_half_angle,
        normalized["z"] / sin_half_angle,
    )
    clamped_half_angle = max_angle / 2.0
    clamped_sin = math.sin(clamped_half_angle)

    return {
        "w": math.cos(clamped_half_angle),
        "x": axis[0] * clamped_sin,
        "y": axis[1] * clamped_sin,
        "z": axis[2] * clamped_sin,
    }


def load_joint_limits(path: Path) -> tuple[float, dict[str, float]]:
    # Joint limits are keyed by chain name so prediction generation can clamp
    # each limb relation differently.
    if not path.exists():
        return DEFAULT_MAX_ANGLE_DEGREES, {}

    default_limit = DEFAULT_MAX_ANGLE_DEGREES
    chain_limits: dict[str, float] = {}
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            chain = row["chain"]
            max_angle = float(row["max_angle_degrees"])
            if chain == "DEFAULT":
                default_limit = max_angle
            else:
                chain_limits[chain] = max_angle

    return default_limit, chain_limits


def clamp_chain_quaternion(
    chain: str,
    quaternion: dict[str, float],
    default_limit: float,
    chain_limits: dict[str, float],
) -> dict[str, float]:
    max_angle = chain_limits.get(chain, default_limit)
    return clamp_quaternion_angle(quaternion, max_angle)


def load_processed_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV has no header: {path}")

        return reader.fieldnames, list(reader)


def save_prediction_csv(fieldnames: list[str], rows: list[dict[str, str]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        path.unlink()

    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def load_processed_frames(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def save_processed_frames(frames: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        path.unlink()

    with path.open("w", encoding="utf-8") as handle:
        json.dump(frames, handle, indent=2)


def add_quaternion_noise_to_rows(rows: list[dict[str, str]], noise_std: float, seed: int | None) -> list[dict[str, str]]:
    random_generator = random.Random(seed)
    noisy_rows: list[dict[str, str]] = []

    for row in rows:
        noisy_row = {"frame_index": row["frame_index"]}

        for column, value in row.items():
            if column == "frame_index":
                continue
            noisy_row[column] = add_quaternion_noise(value, noise_std, random_generator)

        noisy_rows.append(noisy_row)

    return noisy_rows


def create_baseline_prediction(
    sample_id: str,
    processed_dir: Path,
    predictions_dir: Path,
    joint_limits_path: Path,
    noise_std: float,
    seed: int | None,
) -> Path:
    # The non-training path is deliberately simple: start from processed pose
    # targets, optionally add noise, then clamp to plausible joint ranges.
    processed_path = processed_dir / f"processed_{sample_id}.csv"
    if not processed_path.exists():
        raise FileNotFoundError(
            f"Processed recording not found: {processed_path}. "
            f"Run preprocess.py {sample_id} first."
        )

    fieldnames, rows = load_processed_csv(processed_path)
    if noise_std > 0.0:
        quaternion_rows = add_quaternion_noise_to_rows(rows, noise_std, seed)
    else:
        quaternion_rows = rows
    default_limit, chain_limits = load_joint_limits(joint_limits_path)
    prediction_rows: list[dict[str, str]] = []

    for row in quaternion_rows:
        prediction_row = {"frame_index": row["frame_index"]}
        for column, value in row.items():
            if column == "frame_index":
                continue
            prediction_row[column] = format_quaternion(
                clamp_chain_quaternion(column, parse_quaternion(value), default_limit, chain_limits)
            )
        prediction_rows.append(prediction_row)

    prediction_path = predictions_dir / f"prediction_{sample_id}.csv"
    save_prediction_csv(fieldnames, prediction_rows, prediction_path)
    return prediction_path


def require_torch() -> None:
    if torch is None or nn is None or DataLoader is None:
        raise ModuleNotFoundError(
            "PyTorch is required only for training. Install it before using --train, "
            "or run model.py without --train to create baseline predictions."
        )


def frame_to_tensor(frame: dict) -> "torch.Tensor":
    # Flatten centered joints into a dense feature vector suitable for the
    # simple sequence model below.
    require_torch()
    values: list[float] = []
    joints = frame["joints_centered"]
    for joint_id in JOINT_IDS:
        joint = joints.get(str(joint_id), {"x": 0.0, "y": 0.0, "z": 0.0})
        values.extend((joint["x"], joint["y"], joint["z"]))
    return torch.tensor(values, dtype=torch.float32)


class PoseSequenceDataset(Dataset):
    def __init__(self, predictions_dir: Path, processed_dir: Path, window_size: int) -> None:
        # Training samples are sliding windows of predicted pose -> target pose.
        require_torch()
        self.samples: list[tuple[torch.Tensor, torch.Tensor]] = []
        self.window_size = window_size

        for prediction_path in sorted(predictions_dir.glob("prediction_*.json")):
            sample_suffix = prediction_path.stem.removeprefix("prediction_")
            target_path = processed_dir / f"processed_{sample_suffix}.json"
            if not target_path.exists():
                continue

            prediction_frames = load_processed_frames(prediction_path)
            target_frames = load_processed_frames(target_path)
            frame_count = min(len(prediction_frames), len(target_frames))
            if frame_count < window_size:
                continue

            prediction_tensors = [frame_to_tensor(frame) for frame in prediction_frames[:frame_count]]
            target_tensors = [frame_to_tensor(frame) for frame in target_frames[:frame_count]]

            for start_index in range(0, frame_count - window_size + 1):
                input_window = torch.stack(prediction_tensors[start_index:start_index + window_size], dim=0)
                target_window = torch.stack(target_tensors[start_index:start_index + window_size], dim=0)
                self.samples.append((input_window, target_window))

        if not self.samples:
            raise ValueError("No paired training samples found. Run preprocess.py first to create processed and prediction files.")

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, index: int) -> tuple[torch.Tensor, torch.Tensor]:
        return self.samples[index]


if nn is not None:
    class PoseCorrectionModel(nn.Module):
        def __init__(self, input_size: int, hidden_size: int, num_layers: int) -> None:
            super().__init__()
            # A small GRU baseline is enough for experimentation before wiring in
            # real EMG features.
            self.encoder = nn.GRU(
                input_size=input_size,
                hidden_size=hidden_size,
                num_layers=num_layers,
                batch_first=True,
            )
            self.head = nn.Sequential(
                nn.Linear(hidden_size, hidden_size),
                nn.ReLU(),
                nn.Linear(hidden_size, input_size),
            )

        def forward(self, x: "torch.Tensor") -> "torch.Tensor":
            encoded, _ = self.encoder(x)
            return self.head(encoded)
else:
    PoseCorrectionModel = None


def train_model(
    predictions_dir: Path,
    processed_dir: Path,
    output_path: Path,
    window_size: int,
    hidden_size: int,
    num_layers: int,
    batch_size: int,
    epochs: int,
    learning_rate: float,
) -> None:
    # Training is optional and intentionally decoupled from the baseline
    # prediction path so the rest of the pipeline works without PyTorch.
    require_torch()
    dataset = PoseSequenceDataset(predictions_dir, processed_dir, window_size)
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

    input_size = len(JOINT_IDS) * FEATURES_PER_JOINT
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    model = PoseCorrectionModel(input_size=input_size, hidden_size=hidden_size, num_layers=num_layers).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
    loss_fn = nn.MSELoss()

    model.train()
    for epoch_index in range(epochs):
        running_loss = 0.0
        for inputs, targets in dataloader:
            inputs = inputs.to(device)
            targets = targets.to(device)

            optimizer.zero_grad()
            predictions = model(inputs)
            loss = loss_fn(predictions, targets)
            loss.backward()
            optimizer.step()

            running_loss += loss.item() * inputs.size(0)

        epoch_loss = running_loss / len(dataset)
        print(f"epoch {epoch_index + 1}/{epochs}  loss={epoch_loss:.6f}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "model_state_dict": model.state_dict(),
            "input_size": input_size,
            "hidden_size": hidden_size,
            "num_layers": num_layers,
            "window_size": window_size,
            "joint_ids": JOINT_IDS,
        },
        output_path,
    )
    print(f"saved model to {output_path}")


def main() -> int:
    script_directory = Path(__file__).resolve().parent
    data_directory = script_directory / "data"

    parser = argparse.ArgumentParser(description="Create baseline predictions or train a simple PyTorch pose-correction model.")
    parser.add_argument(
        "sample_id",
        nargs="?",
        default="0",
        help="Numeric sample id used for prediction output, e.g. 0 or 1.",
    )
    parser.add_argument("--train", action="store_true", help="Train the model instead of creating a baseline prediction.")
    parser.add_argument("--predictions-dir", default=str(data_directory / "predictions"))
    parser.add_argument("--processed-dir", default=str(data_directory / "recordings" / "processed"))
    parser.add_argument("--joint-limits", default=str(data_directory / "joint_limits.csv"))
    parser.add_argument("--output", default=str(data_directory / "models" / "pose_correction.pt"))
    parser.add_argument("--window-size", type=int, default=30)
    parser.add_argument("--hidden-size", type=int, default=128)
    parser.add_argument("--num-layers", type=int, default=2)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--noise-std", type=float, default=0.0, help="Prediction quaternion noise standard deviation.")
    parser.add_argument("--seed", type=int, default=None, help="Optional random seed for reproducible baseline predictions.")
    args = parser.parse_args()

    if args.train:
        train_model(
            predictions_dir=Path(args.predictions_dir),
            processed_dir=Path(args.processed_dir),
            output_path=Path(args.output),
            window_size=args.window_size,
            hidden_size=args.hidden_size,
            num_layers=args.num_layers,
            batch_size=args.batch_size,
            epochs=args.epochs,
            learning_rate=args.learning_rate,
        )
    else:
        prediction_path = create_baseline_prediction(
            sample_id=args.sample_id,
            processed_dir=Path(args.processed_dir),
            predictions_dir=Path(args.predictions_dir),
            joint_limits_path=Path(args.joint_limits),
            noise_std=args.noise_std,
            seed=args.seed,
        )
        print(f"wrote baseline prediction to {prediction_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

from __future__ import annotations

import argparse
import csv
import random
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


def load_joint_limit_chains(path: Path) -> set[str]:
    if not path.exists():
        raise FileNotFoundError(f"Joint limits not found: {path}")

    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        return {row["chain"] for row in reader}


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


def clamp_normalized_values(values: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(max(0.0, min(1.0, value)) for value in values)


def add_prediction_noise(values: tuple[float, float, float], noise_std: float, random_generator: random.Random) -> tuple[float, float, float]:
    if noise_std <= 0.0:
        return clamp_normalized_values(values)

    return clamp_normalized_values(
        tuple(value + random_generator.gauss(0.0, noise_std) for value in values)
    )


def parse_pose_target(column: str, value: str, valid_columns: set[str]) -> tuple[float, float, float]:
    if column not in valid_columns:
        raise ValueError(f"Pose column {column!r} has no entry in joint_limits.csv")

    return clamp_normalized_values(parse_normalized_limit_values(value))


def create_baseline_prediction(sample_id: str, processed_dir: Path, predictions_dir: Path, joint_limits_path: Path, noise_std: float, seed: int | None) -> Path:
    processed_path = processed_dir / f"processed_{sample_id}.csv"
    if not processed_path.exists():
        raise FileNotFoundError(
            f"Processed recording not found: {processed_path}. "
            f"Run preprocess.py {sample_id} first."
        )

    valid_columns = load_joint_limit_chains(joint_limits_path)
    fieldnames, rows = load_processed_csv(processed_path)
    random_generator = random.Random(seed)
    prediction_rows: list[dict[str, str]] = []

    for row in rows:
        prediction_row = {"frame_index": row["frame_index"]}
        for column, value in row.items():
            if column == "frame_index":
                continue

            values = parse_pose_target(column, value, valid_columns)
            prediction_row[column] = format_normalized_limit_values(
                add_prediction_noise(values, noise_std, random_generator)
            )
        prediction_rows.append(prediction_row)

    prediction_rows = filter_normalized_pose_jitter(prediction_rows)

    prediction_path = predictions_dir / f"prediction_{sample_id}.csv"
    save_prediction_csv(fieldnames, prediction_rows, prediction_path)
    return prediction_path


def main() -> int:
    code_directory = Path(__file__).resolve().parent.parent
    data_directory = code_directory / "data"

    parser = argparse.ArgumentParser(description="Create normalized baseline pose predictions.")
    parser.add_argument(
        "sample_id",
        nargs="?",
        default="0",
        help="Numeric sample id used for prediction output, e.g. 0 or 1.",
    )
    parser.add_argument("--processed-dir", default=str(data_directory / "recordings" / "processed"))
    parser.add_argument("--predictions-dir", default=str(data_directory / "predictions"))
    parser.add_argument("--joint-limits", default=str(data_directory / "joint_limits.csv"))
    parser.add_argument("--noise-std", type=float, default=0.0, help="Noise standard deviation in normalized 0..1 pose space.")
    parser.add_argument("--seed", type=int, default=None, help="Optional random seed for reproducible baseline predictions.")
    args = parser.parse_args()

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

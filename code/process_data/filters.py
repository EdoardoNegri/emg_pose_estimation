import math


NORMALIZED_SPIKE_JUMP = 0.20
NORMALIZED_SPIKE_RETURN = 0.10
NORMALIZED_MEDIAN_JUMP = 0.20
NORMALIZED_MEDIAN_WINDOW_RADIUS = 2
MAX_NORMALIZED_FRAME_STEP = 0.16
LEFT_FOOT_CHAIN_COLUMNS = {"12-13-14", "13-14-15"}
RIGHT_FOOT_CHAIN_COLUMNS = {"16-17-18", "17-18-19"}
FOOT_CHAIN_COLUMNS = LEFT_FOOT_CHAIN_COLUMNS | RIGHT_FOOT_CHAIN_COLUMNS
MAX_FOOT_NORMALIZED_FRAME_STEP = 0.045
FOOT_SMOOTHING_ALPHA = 0.20
RIGHT_FOOT_SMOOTHING_ALPHA = 0.12
MAX_RIGHT_FOOT_NORMALIZED_FRAME_STEP = 0.030


def parse_normalized_limit_values(value: str) -> tuple[float, float, float]:
    components = tuple(float(component) for component in value.split())
    if len(components) != 3:
        raise ValueError(f"Expected 3 normalized rotation values, got {len(components)} in {value!r}")
    return components


def format_normalized_limit_values(values: tuple[float, float, float]) -> str:
    return " ".join(f"{value:.9f}" for value in values)


def normalized_values_distance(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
    return math.sqrt(sum((left[index] - right[index]) ** 2 for index in range(3)))


def filter_normalized_pose_jitter(rows: list[dict[str, str]], spike_jump: float = NORMALIZED_SPIKE_JUMP, spike_return: float = NORMALIZED_SPIKE_RETURN, median_jump: float = NORMALIZED_MEDIAN_JUMP, median_window_radius: int = NORMALIZED_MEDIAN_WINDOW_RADIUS, max_frame_step: float = MAX_NORMALIZED_FRAME_STEP) -> list[dict[str, str]]:
    if len(rows) < 3:
        return rows

    filtered_rows = [dict(row) for row in rows]
    columns = [column for column in rows[0] if column != "frame_index"]

    for row_index in range(1, len(rows) - 1):
        previous_row = rows[row_index - 1]
        current_row = rows[row_index]
        next_row = rows[row_index + 1]
        repaired_row = dict(filtered_rows[row_index])

        for column in columns:
            previous_values = parse_normalized_limit_values(previous_row[column])
            current_values = parse_normalized_limit_values(current_row[column])
            next_values = parse_normalized_limit_values(next_row[column])
            is_local_spike = (
                normalized_values_distance(current_values, previous_values) > spike_jump
                and normalized_values_distance(current_values, next_values) > spike_jump
                and normalized_values_distance(previous_values, next_values) <= spike_return
            )
            if not is_local_spike:
                continue

            repaired_values = tuple(
                (previous_values[index] + next_values[index]) / 2.0
                for index in range(3)
            )
            repaired_row[column] = format_normalized_limit_values(repaired_values)

        filtered_rows[row_index] = repaired_row

    for _ in range(2):
        source_rows = [dict(row) for row in filtered_rows]
        for row_index in range(len(source_rows)):
            window_start = max(0, row_index - median_window_radius)
            window_end = min(len(source_rows), row_index + median_window_radius + 1)
            if window_end - window_start < 3:
                continue

            repaired_row = dict(filtered_rows[row_index])
            for column in columns:
                current_values = parse_normalized_limit_values(source_rows[row_index][column])
                window_values = [
                    parse_normalized_limit_values(source_rows[index][column])
                    for index in range(window_start, window_end)
                ]
                median_values = tuple(
                    sorted(values[component] for values in window_values)[len(window_values) // 2]
                    for component in range(3)
                )
                if normalized_values_distance(current_values, median_values) <= median_jump:
                    continue
                repaired_row[column] = format_normalized_limit_values(median_values)

            filtered_rows[row_index] = repaired_row

    for row_index in range(1, len(filtered_rows)):
        previous_row = filtered_rows[row_index - 1]
        current_row = filtered_rows[row_index]
        repaired_row = dict(current_row)

        for column in columns:
            previous_values = parse_normalized_limit_values(previous_row[column])
            current_values = parse_normalized_limit_values(current_row[column])
            distance = normalized_values_distance(previous_values, current_values)
            if distance <= max_frame_step or distance <= 1e-12:
                continue

            scale = max_frame_step / distance
            limited_values = tuple(
                previous_values[index] + ((current_values[index] - previous_values[index]) * scale)
                for index in range(3)
            )
            repaired_row[column] = format_normalized_limit_values(limited_values)

        filtered_rows[row_index] = repaired_row

    for row_index in range(1, len(filtered_rows)):
        previous_row = filtered_rows[row_index - 1]
        current_row = filtered_rows[row_index]
        repaired_row = dict(current_row)

        for column in FOOT_CHAIN_COLUMNS & set(columns):
            previous_values = parse_normalized_limit_values(previous_row[column])
            current_values = parse_normalized_limit_values(current_row[column])
            smoothing_alpha = RIGHT_FOOT_SMOOTHING_ALPHA if column in RIGHT_FOOT_CHAIN_COLUMNS else FOOT_SMOOTHING_ALPHA
            max_step = MAX_RIGHT_FOOT_NORMALIZED_FRAME_STEP if column in RIGHT_FOOT_CHAIN_COLUMNS else MAX_FOOT_NORMALIZED_FRAME_STEP
            smoothed_values = tuple(
                previous_values[index] + ((current_values[index] - previous_values[index]) * smoothing_alpha)
                for index in range(3)
            )
            distance = normalized_values_distance(previous_values, smoothed_values)
            if distance > max_step:
                scale = max_step / distance
                smoothed_values = tuple(
                    previous_values[index] + ((smoothed_values[index] - previous_values[index]) * scale)
                    for index in range(3)
                )
            repaired_row[column] = format_normalized_limit_values(smoothed_values)

        filtered_rows[row_index] = repaired_row

    return filtered_rows

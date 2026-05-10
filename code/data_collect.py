import argparse
import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path

# Simple IMU data collection and cropping utility.
# It receives MQTT IMU data, writes raw CSV, and crops samples for model training.
# =========================
# Config
# =========================

MQTT_BROKER = "20.25.211.36"
MQTT_PORT = 1883
MQTT_TOPIC = "DATA"

BASE_DIR = Path(__file__).resolve().parent

SAVE_DIR = BASE_DIR / "imu_logs"
CROP_DIR = BASE_DIR / "1D_CNN_Model" / "dataset" / "samples"
CROP_LOG_DIR = BASE_DIR / "crop_logs"

SAVE_DIR.mkdir(parents=True, exist_ok=True)
CROP_DIR.mkdir(parents=True, exist_ok=True)
CROP_LOG_DIR.mkdir(parents=True, exist_ok=True)

CSV_HEADER = [
    "time", "idx", "total",
    "i0_ax", "i0_ay", "i0_az",
    "i0_gx", "i0_gy", "i0_gz",
    "i1_ax", "i1_ay", "i1_az",
    "i1_gx", "i1_gy", "i1_gz",
]

CROP_POINTS = 200
ENERGY_WINDOW_POINTS = 150
POINTS_BEFORE_ENERGY_CENTER = CROP_POINTS // 2
GYRO_SCORE_WEIGHT = 0.02
NORMALIZE_CROPPED_INDEX = True
SAVE_CROP_IMAGES = True

# =========================
# Runtime State
# =========================

current_file = None
current_file_path = None
csv_writer = None
buffer = []
batch_rows = []
BUFFER_SIZE = 100

# =========================
# Preprocess / Crop Utils
# =========================

def _as_float(row, key, default=0.0):
    # Read a numeric field from a row and convert to float.
    try:
        if isinstance(row, dict):
            return float(row[key])
        return float(row[CSV_HEADER.index(key)])
    except (KeyError, TypeError, ValueError, IndexError):
        return default


def _vec_norm(values):
    # Compute Euclidean norm of a value vector.
    return math.sqrt(sum(value * value for value in values))


def _acc(row, imu_prefix):
    # Get accelerometer values for a given IMU prefix.
    return [
        _as_float(row, f"{imu_prefix}_ax"),
        _as_float(row, f"{imu_prefix}_ay"),
        _as_float(row, f"{imu_prefix}_az"),
    ]


def _gyro(row, imu_prefix):
    # Get gyroscope values for a given IMU prefix.
    return [
        _as_float(row, f"{imu_prefix}_gx"),
        _as_float(row, f"{imu_prefix}_gy"),
        _as_float(row, f"{imu_prefix}_gz"),
    ]


def _distance(left, right):
    # Compute the distance between two vectors.
    return _vec_norm([left[i] - right[i] for i in range(len(left))])


def _motion_scores(rows):
    # Compute a motion score per row using acceleration delta and gyro magnitude.
    scores = []
    for index, row in enumerate(rows):
        gyro_score = max(_vec_norm(_gyro(row, "i0")), _vec_norm(_gyro(row, "i1")))

        if index == 0:
            acc_delta = 0.0
        else:
            previous = rows[index - 1]
            acc_delta = (
                _distance(_acc(row, "i0"), _acc(previous, "i0"))
                + _distance(_acc(row, "i1"), _acc(previous, "i1"))
            )

        scores.append(acc_delta + GYRO_SCORE_WEIGHT * gyro_score)
    return scores


def _moving_average(values, window=3):
    # Smooth scores using a simple moving average.
    smoothed = []
    for index in range(len(values)):
        start = max(0, index - window + 1)
        chunk = values[start:index + 1]
        smoothed.append(sum(chunk) / len(chunk))
    return smoothed


def _raw_idx(row):
    # Parse the raw index from a row.
    try:
        return int(float(_as_float(row, "idx")))
    except (TypeError, ValueError):
        return None


def _normalize_cropped_rows(cropped_rows):
    # Reset idx and total fields after cropping when normalization is enabled.
    if not NORMALIZE_CROPPED_INDEX:
        return cropped_rows

    normalized_rows = []
    idx_column = CSV_HEADER.index("idx")
    total_column = CSV_HEADER.index("total")

    for cropped_index, row in enumerate(cropped_rows):
        if isinstance(row, dict):
            normalized_row = dict(row)
            normalized_row["idx"] = cropped_index
            normalized_row["total"] = len(cropped_rows)
        else:
            normalized_row = list(row)
            normalized_row[idx_column] = cropped_index
            normalized_row[total_column] = len(cropped_rows)

        normalized_rows.append(normalized_row)

    return normalized_rows


def find_max_energy_region(rows, energy_window_points=ENERGY_WINDOW_POINTS):
    # Find the window with the highest motion energy in the raw rows.
    if not rows:
        return {
            "scores": [],
            "energy_start": 0,
            "energy_end": 0,
            "energy_center": 0,
            "peak_index": 0,
            "peak_score": 0.0,
            "energy_sum": 0.0,
        }

    scores = _moving_average(_motion_scores(rows), window=3)
    window_size = min(max(1, energy_window_points), len(scores))

    current_sum = sum(scores[:window_size])
    best_sum = current_sum
    best_start = 0

    for start in range(1, len(scores) - window_size + 1):
        current_sum += scores[start + window_size - 1] - scores[start - 1]
        if current_sum > best_sum:
            best_sum = current_sum
            best_start = start

    best_end = best_start + window_size
    local_peak_offset = max(
        range(window_size),
        key=lambda offset: scores[best_start + offset],
    )
    peak_index = best_start + local_peak_offset

    return {
        "scores": scores,
        "energy_start": best_start,
        "energy_end": best_end - 1,
        "energy_center": best_start + window_size // 2,
        "peak_index": peak_index,
        "peak_score": scores[peak_index],
        "energy_sum": best_sum,
    }


def crop_rows_to_motion_window(rows, crop_points=CROP_POINTS):
    # Crop rows around the highest-energy motion window.
    if len(rows) <= crop_points:
        metadata = {
            "algorithm": "max_energy_region",
            "input_rows": len(rows),
            "crop_points": len(rows),
            "crop_start": 0,
            "crop_end": 0 if not rows else len(rows) - 1,
            "raw_start_idx": _raw_idx(rows[0]) if rows else None,
            "raw_end_idx": _raw_idx(rows[-1]) if rows else None,
            "normalized_idx": NORMALIZE_CROPPED_INDEX,
            "energy_window_points": min(ENERGY_WINDOW_POINTS, len(rows)),
            "energy_start": 0,
            "energy_end": 0 if not rows else len(rows) - 1,
            "energy_center": 0,
            "peak_index": 0,
            "peak_score": 0.0,
            "energy_sum": 0.0,
            "note": "input_rows <= crop_points, returned original rows",
        }
        return rows[:], metadata

    energy = find_max_energy_region(rows)
    start = energy["energy_center"] - POINTS_BEFORE_ENERGY_CENTER
    start = max(0, min(start, len(rows) - crop_points))
    end = start + crop_points

    metadata = {
        "algorithm": "max_energy_region",
        "input_rows": len(rows),
        "crop_points": crop_points,
        "crop_start": start,
        "crop_end": end - 1,
        "raw_start_idx": _raw_idx(rows[start]),
        "raw_end_idx": _raw_idx(rows[end - 1]),
        "normalized_idx": NORMALIZE_CROPPED_INDEX,
        "energy_window_points": min(ENERGY_WINDOW_POINTS, len(rows)),
        "energy_start": energy["energy_start"],
        "energy_end": energy["energy_end"],
        "energy_center": energy["energy_center"],
        "points_before_energy_center": POINTS_BEFORE_ENERGY_CENTER,
        "peak_index": energy["peak_index"],
        "peak_score": round(energy["peak_score"], 6),
        "energy_sum": round(energy["energy_sum"], 6),
    }

    return rows[start:end], metadata



def _series_norm(rows, imu_prefix, sensor_type):
    # Get magnitude series for a sensor channel (accel or gyro).
    getter = _acc if sensor_type == "acc" else _gyro
    return [_vec_norm(getter(row, imu_prefix)) for row in rows]


def save_crop_debug_images(rows, metadata, output_path, log_dir=CROP_LOG_DIR):
    """Save two PNG images showing the final crop window on the raw signal.

    The images are debug artifacts only. They do not change the rows written to
    the cropped CSV, so inference still reads the same CSV format as before.
    """
    if not SAVE_CROP_IMAGES:
        return []

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        print("[WARN] matplotlib is not available; skipped crop debug images")
        return []

    if not rows:
        return []

    output_path = Path(output_path)
    log_dir = Path(log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    base_name = output_path.stem
    overview_path = log_dir / f"{base_name}_crop_overview.png"
    signal_path = log_dir / f"{base_name}_crop_signal.png"

    x = list(range(len(rows)))
    crop_start = int(metadata.get("crop_start", 0))
    crop_end = int(metadata.get("crop_end", 0))
    energy_start = int(metadata.get("energy_start", 0))
    energy_end = int(metadata.get("energy_end", 0))
    peak_index = int(metadata.get("peak_index", 0))
    scores = find_max_energy_region(rows).get("scores", [])

    plt.figure(figsize=(12, 5))
    if scores:
        plt.plot(x[:len(scores)], scores, linewidth=1.2, label="motion score")
    plt.axvspan(crop_start, crop_end, alpha=0.25, label="final crop window")
    plt.axvspan(energy_start, energy_end, alpha=0.15, label="max-energy window")
    plt.axvline(peak_index, linestyle="--", linewidth=1.0, label="peak motion")
    plt.title(
        f"Crop overview: rows {crop_start}-{crop_end} / {len(rows)} "
        f"(raw idx {metadata.get('raw_start_idx')}-{metadata.get('raw_end_idx')})"
    )
    plt.xlabel("raw sample index")
    plt.ylabel("motion score")
    plt.legend(loc="best")
    plt.grid(True, alpha=0.25)
    plt.tight_layout()
    plt.savefig(overview_path, dpi=160)
    plt.close()

    plt.figure(figsize=(12, 5))
    plt.plot(x, _series_norm(rows, "i0", "acc"), linewidth=1.0, label="i0 accel magnitude")
    plt.plot(x, _series_norm(rows, "i1", "acc"), linewidth=1.0, label="i1 accel magnitude")
    plt.plot(x, _series_norm(rows, "i0", "gyro"), linewidth=1.0, label="i0 gyro magnitude")
    plt.plot(x, _series_norm(rows, "i1", "gyro"), linewidth=1.0, label="i1 gyro magnitude")
    plt.axvspan(crop_start, crop_end, alpha=0.25, label="final crop window")
    plt.title("Sensor magnitudes with final crop window")
    plt.xlabel("raw sample index")
    plt.ylabel("magnitude")
    plt.legend(loc="best")
    plt.grid(True, alpha=0.25)
    plt.tight_layout()
    plt.savefig(signal_path, dpi=160)
    plt.close()

    print(f"[CROP IMG] {overview_path}")
    print(f"[CROP IMG] {signal_path}")
    return [overview_path, signal_path]

# Write cropped rows to a new CSV file and attach debug image paths.
def write_cropped_rows(rows, output_path, source_path=None):
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    cropped_rows, metadata = crop_rows_to_motion_window(rows)
    output_rows = _normalize_cropped_rows(cropped_rows)

    with open(output_path, "w", newline="", encoding="utf-8") as output_file:
        writer = csv.writer(output_file)
        writer.writerow(CSV_HEADER)
        writer.writerows(output_rows)

    image_paths = save_crop_debug_images(rows, metadata, output_path)
    metadata["crop_image_paths"] = [str(path) for path in image_paths]

    print(
        f"[CROP] {output_path} rows={len(cropped_rows)} "
        f"start={metadata['crop_start']} end={metadata['crop_end']} "
        f"raw_idx={metadata['raw_start_idx']}-{metadata['raw_end_idx']} "
        f"energy={metadata['energy_start']}-{metadata['energy_end']} "
        f"peak={metadata['peak_index']}"
    )

    return metadata


# Crop a raw CSV file by reading it and saving the cropped result.
def crop_csv_file(input_path, output_path=None):
    input_path = Path(input_path)

    if output_path is None:
        output_path = get_next_sample_path()
    else:
        output_path = Path(output_path)

    with open(input_path, newline="", encoding="utf-8") as input_file:
        reader = csv.reader(input_file)
        header = next(reader, None)
        rows = [row for row in reader]

    if header != CSV_HEADER:
        print(f"[WARN] CSV header differs from collector header: {header}")

    write_cropped_rows(rows, output_path, input_path)
    return output_path

# =========================
# Utils
# =========================

def get_next_sample_path():
    # Generate the next sample output path using an incrementing index.
    existing_paths = sorted(CROP_DIR.glob("sample_*.csv"))
    max_index = 0

    for path in existing_paths:
        try:
            number = int(path.stem.split("_")[-1])
            max_index = max(max_index, number)
        except ValueError:
            continue

    next_index = max_index + 1
    return CROP_DIR / f"sample_{next_index:04d}.csv"


def open_new_file(total=None):
    # Start a new CSV file for the next IMU data batch.
    global current_file, current_file_path, csv_writer, batch_rows

    if current_file:
        current_file.close()

    current_file_path = SAVE_DIR / f"imu_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    current_file = open(current_file_path, "w", newline="", encoding="utf-8")
    csv_writer = csv.writer(current_file)
    batch_rows = []

    csv_writer.writerow(CSV_HEADER)

    print(f"[NEW FILE] {current_file_path} total={total}")


def flush_buffer():
    # Flush buffered rows to the current CSV file.
    global buffer

    if buffer and csv_writer:
        csv_writer.writerows(buffer)
        current_file.flush()
        buffer = []

# =========================
# MQTT callbacks
# =========================

def on_connect(client, userdata, flags, rc, properties=None):
    # Subscribe to the IMU topic when MQTT connects.
    print("[MQTT] Connected")
    client.subscribe(MQTT_TOPIC)


def on_message(client, userdata, msg):
    # Process incoming MQTT IMU messages and save rows.
    global buffer, batch_rows

    try:
        payload = msg.payload.decode()
        payload = json.loads(payload)
    except Exception:
        return

    if not payload:
        return

    t = payload.get("type")

    if t == "imu_start":
        print("[START]")
        return

    if t == "imu_end":
        print("[END]")
        flush_buffer()

        if current_file_path and batch_rows:
            crop_path = get_next_sample_path()
            write_cropped_rows(batch_rows, crop_path)

        return

    if t != "imu":
        return

    i0 = payload.get("i0")
    i1 = payload.get("i1")

    if not i0 or not i1:
        return

    if "a" not in i0 or "g" not in i0:
        return

    if "a" not in i1 or "g" not in i1:
        return

    idx = payload.get("idx", 0)
    total = payload.get("total", "")

    if idx == 0:
        open_new_file(total)

    try:
        row = [
            time.time(),
            idx,
            total,

            i0["a"][0], i0["a"][1], i0["a"][2],
            i0["g"][0], i0["g"][1], i0["g"][2],

            i1["a"][0], i1["a"][1], i1["a"][2],
            i1["g"][0], i1["g"][1], i1["g"][2],
        ]

        buffer.append(row)
        batch_rows.append(row)

        if len(buffer) >= BUFFER_SIZE:
            flush_buffer()

    except Exception as e:
        print("[ERROR]", e)

# =========================
# Main
# =========================

def run_collector():
    # Start the MQTT client and run the message loop.
    import paho.mqtt.client as mqtt

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT)
    client.loop_forever()


def main():
    # CLI entry point for collecting or cropping IMU data.
    parser = argparse.ArgumentParser(description="Collect IMU CSV data and crop badminton swing windows.")
    parser.add_argument("--crop", metavar="CSV", help="crop an existing raw CSV to fixed points")
    parser.add_argument("--output", metavar="CSV", help="output path for --crop")
    args = parser.parse_args()

    if args.crop:
        crop_csv_file(args.crop, args.output)
    else:
        run_collector()


if __name__ == "__main__":
    main()

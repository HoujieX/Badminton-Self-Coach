import csv
import json
import os

import numpy as np
import pandas as pd

from clear_multilabel_config import EXPECTED_LEN, FEATURE_COLS, METRIC_COLS


DATASET_DIR = "dataset"
LABELS_CSV = os.path.join(DATASET_DIR, "labels.csv")
OUTPUT_DIR = "processed_clear_multilabel"
RANDOM_SEED = 42

LABEL_TO_VALUE = {
    "bad": 0.0,
    "good": 1.0,
}


def load_one_csv(csv_path):
    df = pd.read_csv(csv_path)

    missing = [c for c in FEATURE_COLS if c not in df.columns]
    if missing:
        raise ValueError(f"{csv_path} is missing columns: {missing}")

    if len(df) != EXPECTED_LEN:
        raise ValueError(f"{csv_path} has {len(df)} rows, expected {EXPECTED_LEN}")

    x = df[FEATURE_COLS].to_numpy(dtype=np.float32)
    return x.T


def encode_labels(row):
    values = []
    for col in METRIC_COLS:
        raw = str(row[col]).strip().lower()
        if raw not in LABEL_TO_VALUE:
            raise ValueError(f"Invalid label for {col}: {row[col]}. Use good or bad.")
        values.append(LABEL_TO_VALUE[raw])
    return values


def read_labels():
    labels = pd.read_csv(LABELS_CSV)

    required_cols = ["file", "action", *METRIC_COLS]
    missing = [c for c in required_cols if c not in labels.columns]
    if missing:
        raise ValueError(f"{LABELS_CSV} is missing columns: {missing}")

    labels = labels[labels["action"].astype(str).str.lower() == "clear"].reset_index(drop=True)
    if labels.empty:
        raise ValueError("No clear action rows found in labels.csv")

    return labels


def build_dataset():
    labels = read_labels()
    x_list = []
    y_list = []
    meta = []

    for _, row in labels.iterrows():
        csv_path = os.path.join(DATASET_DIR, str(row["file"]))
        x_list.append(load_one_csv(csv_path))
        y_list.append(encode_labels(row))
        meta.append({
            "file": str(row["file"]),
            "action": str(row["action"]),
            "labels": {col: str(row[col]) for col in METRIC_COLS},
        })

    X = np.stack(x_list, axis=0).astype(np.float32)
    y = np.array(y_list, dtype=np.float32)
    return X, y, meta


def split_indices(n_samples):
    rng = np.random.default_rng(RANDOM_SEED)
    indices = np.arange(n_samples)
    rng.shuffle(indices)

    n_train = int(n_samples * 0.7)
    n_val = int(n_samples * 0.15)

    train_idx = indices[:n_train]
    val_idx = indices[n_train:n_train + n_val]
    test_idx = indices[n_train + n_val:]
    return train_idx, val_idx, test_idx


def compute_normalization(X_train):
    mean = X_train.mean(axis=(0, 2), keepdims=True)
    std = X_train.std(axis=(0, 2), keepdims=True)
    std = np.where(std < 1e-6, 1.0, std)
    return mean, std


def normalize(X, mean, std):
    return (X - mean) / std


def save_meta(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["split", "file", "action", *METRIC_COLS])
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    X, y, meta = build_dataset()
    train_idx, val_idx, test_idx = split_indices(len(X))

    X_train_raw = X[train_idx]
    mean, std = compute_normalization(X_train_raw)

    splits = {
        "train": (train_idx, normalize(X[train_idx], mean, std), y[train_idx]),
        "val": (val_idx, normalize(X[val_idx], mean, std), y[val_idx]),
        "test": (test_idx, normalize(X[test_idx], mean, std), y[test_idx]),
    }

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    meta_rows = []
    for split_name, (indices, split_x, split_y) in splits.items():
        np.save(os.path.join(OUTPUT_DIR, f"X_{split_name}.npy"), split_x)
        np.save(os.path.join(OUTPUT_DIR, f"y_{split_name}.npy"), split_y)

        for idx in indices:
            item = meta[int(idx)]
            row = {
                "split": split_name,
                "file": item["file"],
                "action": item["action"],
            }
            row.update(item["labels"])
            meta_rows.append(row)

    save_meta(os.path.join(OUTPUT_DIR, "split_meta.csv"), meta_rows)

    with open(os.path.join(OUTPUT_DIR, "norm.json"), "w", encoding="utf-8") as f:
        json.dump({
            "action": "clear",
            "feature_cols": FEATURE_COLS,
            "metric_cols": METRIC_COLS,
            "label_to_value": LABEL_TO_VALUE,
            "mean": mean.squeeze().tolist(),
            "std": std.squeeze().tolist(),
            "expected_len": EXPECTED_LEN,
        }, f, ensure_ascii=False, indent=2)

    print("Saved clear multilabel dataset:")
    for split_name, (_, split_x, split_y) in splits.items():
        print(f"{split_name}: X={split_x.shape}, y={split_y.shape}")
    print(f"Output dir: {OUTPUT_DIR}")

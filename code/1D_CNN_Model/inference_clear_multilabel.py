import argparse
import glob
import json
import os

import numpy as np
import pandas as pd
import torch

from clear_multilabel_config import METRIC_COLS
from clear_multilabel_model import ClearMultiLabelCNN, logits_to_predictions


DATA_DIR = "processed_clear_multilabel"
MODEL_PATH = "clear_multilabel_model.pth"


def load_norm():
    with open(os.path.join(DATA_DIR, "norm.json"), "r", encoding="utf-8") as f:
        norm = json.load(f)

    n_features = len(norm["feature_cols"])
    mean = np.array(norm["mean"], dtype=np.float32).reshape(1, n_features, 1)
    std = np.array(norm["std"], dtype=np.float32).reshape(1, n_features, 1)
    return norm, mean, std


def load_model(metric_cols):
    checkpoint = torch.load(MODEL_PATH, map_location="cpu", weights_only=True)
    model = ClearMultiLabelCNN(num_outputs=len(metric_cols))
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    threshold = float(checkpoint.get("threshold", 0.8))
    return model, threshold


def load_csv(csv_path, feature_cols, expected_len):
    df = pd.read_csv(csv_path)
    missing = [c for c in feature_cols if c not in df.columns]
    if missing:
        raise ValueError(f"{csv_path} is missing columns: {missing}")
    if len(df) != expected_len:
        raise ValueError(f"{csv_path} has {len(df)} rows, expected {expected_len}")

    x = df[feature_cols].to_numpy(dtype=np.float32).T
    return np.expand_dims(x, axis=0)


def predict_one(model, csv_path, norm, mean, std, threshold):
    x = load_csv(csv_path, norm["feature_cols"], norm["expected_len"])
    x = (x - mean) / std
    x = torch.tensor(x, dtype=torch.float32)

    with torch.no_grad():
        logits = model(x)
        preds, probs = logits_to_predictions(logits, threshold=threshold)

    results = {}
    for i, metric in enumerate(norm["metric_cols"]):
        prob_good = float(probs[0, i].item())
        label = "good" if int(preds[0, i].item()) == 1 else "bad"
        results[metric] = {
            "label": label,
            "prob_good": prob_good,
        }
    return results


def iter_csv_paths(input_path):
    if os.path.isdir(input_path):
        return sorted(glob.glob(os.path.join(input_path, "*.csv")))
    return [input_path]


def main():
    parser = argparse.ArgumentParser(description="Run clear-action multilabel inference.")
    parser.add_argument(
        "input",
        nargs="?",
        default="inference_samples",
        help="A CSV file or a directory containing CSV files.",
    )
    args = parser.parse_args()

    norm, mean, std = load_norm()
    model, threshold = load_model(norm["metric_cols"])

    for csv_path in iter_csv_paths(args.input):
        results = predict_one(model, csv_path, norm, mean, std, threshold)
        print(f"\n{csv_path}")
        for metric, item in results.items():
            print(f"  {metric}: {item['label']} (prob_good={item['prob_good']:.3f})")


if __name__ == "__main__":
    main()

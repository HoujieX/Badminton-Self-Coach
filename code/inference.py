import argparse
import csv
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import torch

import data_collect


BASE_DIR = Path(__file__).resolve().parent
MODEL_DIR = BASE_DIR / "1D_CNN_Model"
SAMPLES_DIR = BASE_DIR / "samples"

MQTT_RESULT_TOPIC = "RESULT"

sys.path.insert(0, str(MODEL_DIR))

from clear_multilabel_model import ClearMultiLabelCNN, logits_to_predictions  # noqa: E402


def _load_norm():
    norm_path = MODEL_DIR / "processed_clear_multilabel" / "norm.json"
    with open(norm_path, "r", encoding="utf-8") as f:
        norm = json.load(f)

    n_features = len(norm["feature_cols"])
    mean = np.array(norm["mean"], dtype=np.float32).reshape(1, n_features, 1)
    std = np.array(norm["std"], dtype=np.float32).reshape(1, n_features, 1)
    return norm, mean, std


def _load_model(metric_cols):
    model_path = MODEL_DIR / "clear_multilabel_model.pth"
    checkpoint = torch.load(model_path, map_location="cpu", weights_only=True)
    model = ClearMultiLabelCNN(num_outputs=len(metric_cols))
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    threshold = float(checkpoint.get("threshold", 0.7))
    return model, threshold


def _load_cropped_csv(csv_path, feature_cols, expected_len):
    rows = []
    with open(csv_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        missing = [col for col in feature_cols if col not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"{csv_path} is missing columns: {missing}")

        for row in reader:
            rows.append([float(row[col]) for col in feature_cols])

    if len(rows) != expected_len:
        raise ValueError(f"{csv_path} has {len(rows)} rows, expected {expected_len}")

    x = np.array(rows, dtype=np.float32).T
    return np.expand_dims(x, axis=0)


def predict_csv(csv_path):
    norm, mean, std = _load_norm()
    model, threshold = _load_model(norm["metric_cols"])

    x = _load_cropped_csv(csv_path, norm["feature_cols"], norm["expected_len"])
    x = (x - mean) / std
    x = torch.tensor(x, dtype=torch.float32)

    with torch.no_grad():
        logits = model(x)
        preds, probs = logits_to_predictions(logits, threshold=threshold)

    results = {}
    for i, metric in enumerate(norm["metric_cols"]):
        prob_good = float(probs[0, i].item())
        label = "good" if int(preds[0, i].item()) == 1 else "bad"
        results[metric] = {"label": label, "prob_good": prob_good}

    apply_overall_rule(results)
    return results


def apply_overall_rule(results):
    sub_metrics = [
        "start_action",
        "finish_action",
        "continuity",
        "offset",
        "wrist_action",
    ]

    missing = [metric for metric in sub_metrics if metric not in results]
    if missing:
        raise ValueError(f"Cannot apply overall rule, missing metrics: {missing}")

    overall_good = all(results[metric]["label"] == "good" for metric in sub_metrics)

    results["overall"] = {
        "label": "good" if overall_good else "bad",
        "prob_good": None,
        "source": "rule",
    }


def print_results(csv_path, results):
    print(f"\n{csv_path}")
    for metric, item in results.items():
        if item.get("source") == "rule":
            print(f"  {metric}: {item['label']} (all sub-metrics good)")
        else:
            print(f"  {metric}: {item['label']} (prob_good={item['prob_good']:.3f})")


def crop_raw_csv(raw_csv_path, output_path=None):
    SAMPLES_DIR.mkdir(parents=True, exist_ok=True)

    if output_path is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = SAMPLES_DIR / f"inference_{stamp}_crop{data_collect.CROP_POINTS}.csv"
    else:
        output_path = Path(output_path)

    return data_collect.crop_csv_file(raw_csv_path, output_path)


def infer_existing_csv(raw_csv_path, already_cropped=False):
    if already_cropped:
        csv_path = Path(raw_csv_path)
    else:
        csv_path = crop_raw_csv(raw_csv_path)

    results = predict_csv(csv_path)
    print_results(csv_path, results)

    return csv_path, results


class InferenceCollector:
    def __init__(self, broker, port, topic, result_topic=MQTT_RESULT_TOPIC, once=False):
        self.broker = broker
        self.port = port
        self.topic = topic
        self.result_topic = result_topic
        self.once = once

        self.rows = []
        self.started = False
        self.output_path = None
        self.batch_count = 0

    def on_connect(self, client, userdata, flags, rc, properties=None):
        print("[MQTT] Connected")
        client.subscribe(self.topic)
        print(f"[MQTT] Subscribed to topic {self.topic}")
        print(f"[MQTT] Inference results will be published to topic {self.result_topic}")

    def publish_results(self, client, csv_path, results, crop_metadata=None):
        payload = {
            "type": "inference_result",
            "csv_path": str(csv_path),
            "timestamp": datetime.now().isoformat(),
            "results": results,
        }

        if crop_metadata:
            payload["crop_metadata"] = crop_metadata
            payload["crop_image_paths"] = crop_metadata.get("crop_image_paths", [])

        client.publish(
            self.result_topic,
            json.dumps(payload),
            qos=0,
            retain=False,
        )

        print(f"[MQTT] Published result to topic {self.result_topic}")

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
        except Exception:
            return

        msg_type = payload.get("type")

        if msg_type == "imu_start":
            self.rows = []
            self.started = True
            print("[START]")
            return

        if msg_type == "imu_end":
            print("[END]")

            if self.rows:
                self.batch_count += 1
                SAMPLES_DIR.mkdir(parents=True, exist_ok=True)

                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.output_path = SAMPLES_DIR / (
                    f"inference_{stamp}_{self.batch_count:04d}_crop{data_collect.CROP_POINTS}.csv"
                )

                crop_metadata = data_collect.write_cropped_rows(self.rows, self.output_path)

                try:
                    results = predict_csv(self.output_path)
                    print_results(self.output_path, results)
                    self.publish_results(client, self.output_path, results, crop_metadata=crop_metadata)

                except Exception as e:
                    error_payload = {
                        "type": "inference_error",
                        "csv_path": str(self.output_path),
                        "timestamp": datetime.now().isoformat(),
                        "error": str(e),
                        "crop_metadata": crop_metadata,
                        "crop_image_paths": crop_metadata.get("crop_image_paths", []) if crop_metadata else [],
                    }

                    client.publish(
                        self.result_topic,
                        json.dumps(error_payload),
                        qos=0,
                        retain=False,
                    )

                    print("[ERROR]", e)
                    print(f"[MQTT] Published error to topic {self.result_topic}")

            self.rows = []
            self.started = False

            if self.once:
                client.disconnect()

            return

        if msg_type != "imu":
            return

        i0 = payload.get("i0")
        i1 = payload.get("i1")

        if not i0 or not i1:
            return

        if "a" not in i0 or "g" not in i0:
            return

        if "a" not in i1 or "g" not in i1:
            return

        row = [
            time.time(),
            payload.get("idx", 0),
            payload.get("total", ""),

            i0["a"][0], i0["a"][1], i0["a"][2],
            i0["g"][0], i0["g"][1], i0["g"][2],

            i1["a"][0], i1["a"][1], i1["a"][2],
            i1["g"][0], i1["g"][1], i1["g"][2],
        ]

        self.rows.append(row)

    def run(self):
        import paho.mqtt.client as mqtt

        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        client.on_connect = self.on_connect
        client.on_message = self.on_message

        client.connect(self.broker, self.port)
        client.loop_forever()


def main():
    parser = argparse.ArgumentParser(description="Collect/crop IMU samples and run clear-action inference.")

    parser.add_argument("--csv", help="crop an existing raw CSV and run inference")
    parser.add_argument("--cropped", help="run inference on an already-cropped 200-point CSV")
    parser.add_argument("--once", action="store_true", help="stop after one live MQTT sample")

    parser.add_argument("--broker", default=data_collect.MQTT_BROKER)
    parser.add_argument("--port", type=int, default=data_collect.MQTT_PORT)
    parser.add_argument("--topic", default=data_collect.MQTT_TOPIC)
    parser.add_argument("--result-topic", default=MQTT_RESULT_TOPIC)

    args = parser.parse_args()

    if args.csv:
        infer_existing_csv(args.csv, already_cropped=False)
        return

    if args.cropped:
        infer_existing_csv(args.cropped, already_cropped=True)
        return

    collector = InferenceCollector(
        args.broker,
        args.port,
        args.topic,
        result_topic=args.result_topic,
        once=args.once,
    )

    collector.run()


if __name__ == "__main__":
    main()
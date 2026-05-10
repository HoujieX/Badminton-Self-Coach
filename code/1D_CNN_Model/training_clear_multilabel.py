import os

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset

from clear_multilabel_config import METRIC_COLS
from clear_multilabel_model import ClearMultiLabelCNN, logits_to_predictions


DATA_DIR = "processed_clear_multilabel"
MODEL_PATH = "clear_multilabel_model.pth"
BATCH_SIZE = 8
EPOCHS = 40
LR = 1e-3


def load_split(split):
    X = np.load(os.path.join(DATA_DIR, f"X_{split}.npy"))
    y = np.load(os.path.join(DATA_DIR, f"y_{split}.npy"))
    X = torch.tensor(X, dtype=torch.float32)
    y = torch.tensor(y, dtype=torch.float32)
    return X, y


def make_loader(split, shuffle):
    X, y = load_split(split)
    return DataLoader(TensorDataset(X, y), batch_size=BATCH_SIZE, shuffle=shuffle)


def evaluate(model, loader, criterion):
    model.eval()
    total_loss = 0.0
    total_items = 0
    exact_match = 0
    metric_correct = torch.zeros(len(METRIC_COLS), dtype=torch.long)
    metric_total = 0

    with torch.no_grad():
        for xb, yb in loader:
            logits = model(xb)
            loss = criterion(logits, yb)
            preds, _ = logits_to_predictions(logits)
            targets = yb.long()

            total_loss += loss.item() * xb.size(0)
            total_items += xb.size(0)
            exact_match += (preds == targets).all(dim=1).sum().item()
            metric_correct += (preds == targets).sum(dim=0).cpu()
            metric_total += xb.size(0)

    avg_loss = total_loss / max(total_items, 1)
    exact_acc = exact_match / max(total_items, 1)
    metric_acc = (metric_correct.float() / max(metric_total, 1)).tolist()
    return avg_loss, exact_acc, metric_acc


def main():
    train_loader = make_loader("train", shuffle=True)
    val_loader = make_loader("val", shuffle=False)

    model = ClearMultiLabelCNN(num_outputs=len(METRIC_COLS))
    criterion = nn.BCEWithLogitsLoss()
    optimizer = optim.Adam(model.parameters(), lr=LR, weight_decay=1e-4)

    best_val_exact = -1.0
    best_val_loss = float("inf")

    for epoch in range(1, EPOCHS + 1):
        model.train()
        train_loss_sum = 0.0
        train_items = 0

        for xb, yb in train_loader:
            logits = model(xb)
            loss = criterion(logits, yb)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            train_loss_sum += loss.item() * xb.size(0)
            train_items += xb.size(0)

        train_loss = train_loss_sum / max(train_items, 1)
        val_loss, val_exact, val_metric_acc = evaluate(model, val_loader, criterion)

        metric_text = ", ".join(
            f"{name}={acc:.2f}" for name, acc in zip(METRIC_COLS, val_metric_acc)
        )
        print(
            f"epoch={epoch:02d} "
            f"train_loss={train_loss:.4f} "
            f"val_loss={val_loss:.4f} "
            f"val_exact={val_exact:.4f} "
            f"val_metric_acc: {metric_text}"
        )

        should_save = val_exact > best_val_exact or (
            val_exact == best_val_exact and val_loss < best_val_loss
        )
        if should_save:
            best_val_exact = val_exact
            best_val_loss = val_loss
            torch.save({
                "model_state_dict": model.state_dict(),
                "metric_cols": METRIC_COLS,
                "threshold": 0.8,
            }, MODEL_PATH)
            print(f"Saved best model: {MODEL_PATH}")

    print("Training complete.")


if __name__ == "__main__":
    main()

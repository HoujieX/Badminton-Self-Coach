import torch
import torch.nn as nn

from clear_multilabel_config import FEATURE_COLS, METRIC_COLS


class ClearMultiLabelCNN(nn.Module):
    def __init__(self, in_channels=None, num_outputs=len(METRIC_COLS)):
        super().__init__()
        if in_channels is None:
            in_channels = len(FEATURE_COLS)
        self.features = nn.Sequential(
            nn.Conv1d(in_channels, 32, kernel_size=5, padding=2),
            nn.BatchNorm1d(32),
            nn.ReLU(),
            nn.MaxPool1d(2),
            nn.Dropout(0.15),

            nn.Conv1d(32, 64, kernel_size=5, padding=2),
            nn.BatchNorm1d(64),
            nn.ReLU(),
            nn.MaxPool1d(2),
            nn.Dropout(0.20),

            nn.Conv1d(64, 128, kernel_size=3, padding=1),
            nn.BatchNorm1d(128),
            nn.ReLU(),
            nn.AdaptiveAvgPool1d(1),
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Dropout(0.25),
            nn.Linear(64, num_outputs),
        )

    def forward(self, x):
        x = self.features(x)
        return self.classifier(x)


def logits_to_predictions(logits, threshold=0.5):
    probs = torch.sigmoid(logits)
    preds = (probs >= threshold).long()
    return preds, probs

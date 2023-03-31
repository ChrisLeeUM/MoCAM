import torch
from torch import nn


class DrivingModel(nn.Module):
    def __init__(self) -> None:
        super(DrivingModel, self).__init__()

        self.network = nn.Sequential(
            nn.Conv2d(3, 24, 5, 2),
            nn.BatchNorm2d(24),
            nn.ReLU(),
            nn.Conv2d(24, 36, 5, 2),
            nn.BatchNorm2d(36),
            nn.ReLU(),
            nn.Conv2d(36, 48, 5, 2),
            nn.BatchNorm2d(48),
            nn.ReLU(),
            nn.Conv2d(48, 64, 3),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            nn.Conv2d(64, 64, 3),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            nn.Flatten(),
            nn.Linear(1152, 1164),
            nn.Linear(1164, 100),
            nn.Linear(100, 50),
            nn.Linear(50, 10),
            nn.Linear(10, 1),
        )

    def forward(self, x):
        return self.network(x)

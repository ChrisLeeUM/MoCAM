#! /usr/bin/python

import cv2
import numpy as np
from cv_bridge import CvBridge
import torch
from torchvision import transforms
import torch.optim as optim
from ai_pilot_model import DrivingModel

#import the source files path
CHECKPOINT_PATH = "/home/chris/catkin_ws/src/ai_control/src/models/main.tar"
DATA_PATH = "/home/chris/catkin_ws/src/ai_control/src/datasets/original/"


def load_checkpoint(model: DrivingModel, optimizer: optim.Optimizer = None):
    checkpoint = torch.load(CHECKPOINT_PATH)

    model.load_state_dict(checkpoint["model_state_dict"])

    if optimizer != None:
        optimizer.load_state_dict(checkpoint["optimizer_state_dict"])


dataset_transforms = transforms.Compose([
    transforms.Resize([66, 200]),
    transforms.ToTensor(),
])

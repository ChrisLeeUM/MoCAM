import keyboard
from torch.utils.data import DataLoader
import torch
from data import VehicleDataset
from train import DATA_PATH, dataset_transforms, load_checkpoint
from model import DrivingModel


dataset = VehicleDataset(DATA_PATH, dataset_transforms)
train_loader = DataLoader(dataset)

quit = False


def on_quit():
    global quit
    quit = True


keyboard.on_press_key("q", lambda _: on_quit())

model = DrivingModel()
load_checkpoint(model)

device = torch.device("cuda:0")
model.to(device)

cnt_correct = 0
cnt_total   = 0

for i, data in enumerate(train_loader, 0):
    if quit:
        break

    label, image = data
    print("For", dataset.dataset[i][0])
    print("Expected", dataset.dataset[i][1])
    print("And got", model(image).item())

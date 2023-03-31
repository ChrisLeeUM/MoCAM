from itertools import islice
from torch import nn
import torch
from torch.serialization import save
from torch.utils.data import DataLoader
from torchvision import datasets, transforms
import torch.optim as optim
import sys
from model import DrivingModel
import keyboard
from data import VehicleDataset


CHECKPOINT_PATH = "models/main.tar"
DATA_PATH = "datasets/original/"


def should_resume():
    return "--resume" in sys.argv or "-r" in sys.argv


def save_checkpoint(model: DrivingModel, optimizer: optim.Optimizer):
    torch.save({
        'model_state_dict': model.state_dict(),
        'optimizer_state_dict': optimizer.state_dict(),
    }, CHECKPOINT_PATH)


def load_checkpoint(model: DrivingModel, optimizer: optim.Optimizer = None):
    checkpoint = torch.load(CHECKPOINT_PATH)

    model.load_state_dict(checkpoint["model_state_dict"])

    if optimizer != None:
        optimizer.load_state_dict(checkpoint["optimizer_state_dict"])


dataset_transforms = transforms.Compose([
    transforms.Resize([66, 200]),
    transforms.ToTensor(),
    # transforms.Normalize(0.5, 0.5),
])


def train(model: DrivingModel, optimizer: optim.Optimizer):
    quit = False

    def on_quit():
        nonlocal quit
        quit = True

    keyboard.on_press_key("q", lambda _: on_quit())

    criterion = nn.MSELoss()

    dataset = VehicleDataset(DATA_PATH, dataset_transforms)
    train_loader = DataLoader(dataset, batch_size=50)

    for epoch in range(10):

        running_loss = 0.0

        for i, data in enumerate(train_loader, 0):
            if quit:
                save_checkpoint(model, optimizer)
                return

            # get the inputs; data is a list of [inputs, labels]
            label, image = data

            # zero the parameter gradients
            optimizer.zero_grad()

            # forward + backward + optimize
            outputs = model(image)
            loss = criterion(outputs, label)
            loss.backward()
            optimizer.step()

            # print statistics
            running_loss += loss.item()
            if i % 10 == 0:    # print every 2000 mini-batches
                print('[%d, %5d] loss: %.7f' %
                      (epoch + 1, i + 1, running_loss / (2000)))
                running_loss = 0.0

                save_checkpoint(model, optimizer)

    print('Finished Training')


if __name__ == "__main__":
    model = DrivingModel()
    optimizer = optim.Adam(model.parameters())

    if should_resume():
        load_checkpoint(model, optimizer)

    device = torch.device("cuda:0")
    model.to(device)
    model.train(True)

    train(
        model,
        optimizer,
    )

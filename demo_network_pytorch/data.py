from os import path
import math
import torch
from torch.utils.data import random_split, Dataset
from PIL import Image


class VehicleDataset(Dataset):
    def __init__(self, main_dir, transform) -> None:
        self.main_dir = main_dir
        self.dataset = self.get_dataset()
        self.transform = transform

# Origin method
    # def get_dataset(self):
    #     result = []

    #     with open(path.join(self.main_dir, "data.txt")) as f:
    #         for line in f:
    #             line_values = line.split(",")
    #             line_values = line_values[0].split()

    #             image_path = line_values[0]

    #             # the paper by Nvidia uses the inverse of the turning radius,
    #             # but steering wheel angle is proportional to the inverse of turning radius
    #             # so the steering wheel angle in radians is used as the output
    #             angle = float(line_values[1]) * math.pi / 180

    #             entry = (image_path, angle)

    #             result.append(entry)

    #     return result
# Modified method
    def get_dataset(self):
        result = []

        with open(path.join(self.main_dir, "mocam_data_v02.txt")) as f:
            for line in f:
                line_values = line.split()

                image_path = 'img' + str(line_values[0])
                angle = float(line_values[1])

                entry = (image_path, angle)

                result.append(entry)

        return result

    def __len__(self):
        return len(self.dataset)

    def __getitem__(self, index):
        relative_path, angle = self.dataset[index]
        full_path = path.join(self.main_dir, "mocam_data_v02", relative_path)
        image = Image.open(full_path).convert("RGB")
        device = torch.device("cuda:0")

        image_tensor = self.transform(image).to(device)
        angle_tensor = torch.tensor([angle]).to(device)

        return (angle_tensor, image_tensor)

# FRC2023

Sonic Howl's robot code repository for FRC 2023

### [Github => Discord Webhook](https://discord.com/api/webhooks/1080990460035747840/M5FLqzQWFm_lLNd7CunalWq13Erbpta00Gxa96MRn2Me4PcqpATi5P8xJlI8GTIhtEVm)

## Getting Strated

1. **Installing WPILib:** In order to get started, you will need to install an IDE along with the WPILib library. To do this, follow the steps on the [WPIlib documentation site](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html).

2. **Installing RoboPy (Python library):** Our robot code this year will be written in python. Because of this, we will also need to install the RoboPy python library. To do this, follow the steps on the [RoboPy setup documentation site](https://robotpy.readthedocs.io/en/stable/install/computer.html#install-computer).

3. **Running Robot Code:** To run the robot code, type `py -3 robot.py deploy` into the terminal. To run the code in the simulator, type `py -3 robot.py sim` into the terminal.

## Install Commands

`pip install -U black mypy robotpy robotpy[rev] robotpy[navx] robotpy[ctre] robotpy[pathplannerlib] robotpy[photonvision] robotpy[commands2] robotpy[apriltag]`

# Install Commands RoboRIO

### Python

`python -m robotpy_installer download-python`

`python -m robotpy_installer install-python`

### Robotpy deps

`python -m robotpy_installer download robotpy robotpy[rev] robotpy[navx] robotpy[ctre] robotpy[pathplannerlib] robotpy[photonvision] robotpy[commands2] robotpy[apriltag]`

`python -m robotpy_installer install robotpy robotpy[rev] robotpy[navx] robotpy[ctre] robotpy[pathplannerlib] robotpy[photonvision] robotpy[commands2] robotpy[apriltag]`

OR

`python -m robotpy_installer download robotpy[all]`

`python -m robotpy_installer install robotpy[all]`

# Simulate robot code

`python robot.py sim`

# Deploy to robot

`python robot.py deploy`

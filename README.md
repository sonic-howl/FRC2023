## Install Commands

py -3 -m pip install -U robotpy robotpy[rev] robotpy[navx] robotpy[ctre] robotpy[pathplannerlib] robotpy[photonvision] robotpy[commands2] robotpy[apriltag]

# Install Commands RoboRIO

py -3 -m robotpy_installer download-python
py -3 -m robotpy_installer install-python

py -3 -m robotpy_installer download robotpy
py -3 -m robotpy_installer install robotpy

py -3 -m robotpy_installer download robotpy robotpy[rev] robotpy[navx] robotpy[ctre] robotpy[pathplannerlib] robotpy[photonvision] robotpy[commands2] robotpy[apriltag]
py -3 -m robotpy_installer install robotpy robotpy[rev] robotpy[navx] robotpy[ctre] robotpy[pathplannerlib] robotpy[photonvision] robotpy[commands2] robotpy[apriltag]

# Simulate robot code

py -3 robot.py sim

# Deploy to robot

py -3 robot.py deploy

## Install Commands

`pip install -U black mypy robotpy robotpy[rev] robotpy[navx] robotpy[ctre] robotpy[pathplannerlib] robotpy[photonvision] robotpy[commands2] robotpy[apriltag]`

# Install Commands RoboRIO

### Python
`python -m robotpy_installer download-python`

`python -m robotpy_installer install-python`


### Robotpy deps
`python -m robotpy_installer download robotpy robotpy[rev] robotpy[navx] robotpy[ctre] robotpy[pathplannerlib] robotpy[photonvision] robotpy[commands2] robotpy[apriltag]`

`python -m robotpy_installer install robotpy robotpy[rev] robotpy[navx] robotpy[ctre] robotpy[pathplannerlib] robotpy[photonvision] robotpy[commands2] robotpy[apriltag]`

# Simulate robot code

`python robot.py sim`

# Deploy to robot

`python robot.py deploy`

from threading import Thread


def print_async(*args, **kwargs) -> None:
    """print in a new thread"""
    Thread(target=print, args=args, kwargs=kwargs).start()


def sign(x: float) -> float:
    """return the sign of x"""
    return -1 if x < 0 else 1


def RPMToFalcon(RPM: float, gearRatio: float) -> float:
    """convert RPM to falcon encoder units per 100ms"""
    motorRPM = RPM * gearRatio
    sensorCounts = motorRPM * (2048.0 / 600.0)
    return sensorCounts


def MPSToFalcon(velocity: float, circumference: float, gearRatio: float) -> float:
    """convert meters per second to falcon encoder units per 100ms"""
    wheelRPM = (velocity * 60) / circumference
    wheelVelocity = RPMToFalcon(wheelRPM, gearRatio)
    return wheelVelocity


import math
from constants import SwerveConstants

print(
    "MPSToFalcon",
    MPSToFalcon(
        4,
        SwerveConstants.kWheelCircumferenceMeters,
        SwerveConstants.kDrivingMotorReduction,
    ),
)

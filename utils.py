from threading import Thread

from constants import SwerveConstants


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


def falconToMeters(
    positionCounts: float, circumference: float, gearRatio: float
) -> float:
    """convert falcon encoder units per 100ms to meters"""
    return positionCounts * (circumference / (gearRatio * 2048.0))


def MetersToFalcon(meters: float, circumference: float, gearRatio: float) -> float:
    """convert meters to falcon encoder units per 100ms"""
    return meters / (circumference / (gearRatio * 2048.0))

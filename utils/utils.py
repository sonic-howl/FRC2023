from threading import Thread

from constants import Constants


def printAsync(*args, **kwargs) -> None:
    """print in a new thread"""
    Thread(target=print, args=args, kwargs=kwargs).start()


def sign(x: float) -> float:
    """return the sign of x"""
    return -1 if x < 0 else 1


def dz(x: float, dz: float = Constants.controller_deadzone):
    return x if abs(x) > dz else 0

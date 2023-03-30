from wpilib import Timer
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose2d, Rotation2d


class LLTable:
    _instance = None

    def __init__(self):
        if LLTable._instance is not None:
            raise Exception("This class is a singleton!")
        else:
            LLTable._instance = self

            self.setup()

    @staticmethod
    def getInstance():
        if LLTable._instance is None:
            LLTable()
        return LLTable._instance

    def setup(self):
        ntInstance = NetworkTableInstance.getDefault()
        self.limelightTable = ntInstance.getTable("limelight")
        self.tv = self.limelightTable.getIntegerTopic("tv").subscribe(0)
        self.tx = self.limelightTable.getIntegerTopic("tx").subscribe(0)
        self.ta = self.limelightTable.getDoubleTopic("ta").subscribe(0.0)
        self.botpose = self.limelightTable.getDoubleArrayTopic("botpose").subscribe(
            [0, 0, 0, 0, 0, 0, 0]
        )
        self.camMode = self.limelightTable.getIntegerTopic("camMode").getEntry(0)
        self.ledMode = self.limelightTable.getIntegerTopic("ledMode").getEntry(0)
        self.ledMode.set(1)  # force off

    def getTv(self):
        return self.tv.get()

    def getTx(self):
        return self.tx.get()

    def getTxScaled(self):
        """Returns tx scaled on the interval [-1, 1]"""
        return self.getTx() / 29.8

    def getTa(self):
        return self.ta.get()

    def getCamMode(self):
        return self.camMode.get()

    def setCamMode(self, mode: int):
        self.camMode.set(mode)

    def toggleCamMode(self):
        self.setCamMode(0 if self.getCamMode() == 1 else 1)

    def getPose2dAndVisionTs(self):
        x, y, z, pitch, yaw, roll, totalLatency = self.botpose.get()
        return (
            Pose2d(x, y, Rotation2d.fromDegrees(yaw)),
            float(Timer.getFPGATimestamp() - (totalLatency / 1000.0)),
        )

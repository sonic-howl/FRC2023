from constants.RobotConstants import RobotConstants
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile


class VisionConstants:
    minAprilTagArea = 0.1  # TODO calibrate to find 1m away
    aprilTagPipelineID = 0
    reflectiveTapePipelineID = 1

    # TODO calibrate
    kPVisionDrive = 1.2
    kIVisionDrive = 0
    kDVisionDrive = 0.02
    kMaxVisionMetersPerSecond = 0.75
    kMaxVisionMetersPerSecondSquared = 0.75

    xyVisionPID = ProfiledPIDController(
        kPVisionDrive,
        kIVisionDrive,
        kDVisionDrive,
        TrapezoidProfile.Constraints(
            kMaxVisionMetersPerSecond, kMaxVisionMetersPerSecondSquared
        ),
        period=RobotConstants.period,
    )
    """x and y are the same since we're using swerve, hence xyVisionPID"""

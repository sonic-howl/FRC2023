from enum import Enum

import rev
import wpilib

from constants.GameConstants import GamePieceType


class ArmConstants:
    class AngleType(Enum):
        kStow = 0
        kFloor = 1
        kUpperFeedStation = 2
        # kLowerFeedStation = 3
        kGridS1 = 4
        kGridS2 = 5
        kGridS3 = 6

    class SubsystemType(Enum):
        kArm = 0
        kClaw = 1

    # ! the values here can be disregarded if they are set by the the RIO's preferences. Toggle below.
    angles: dict[GamePieceType, dict[AngleType, dict[SubsystemType, float]]] = {
        GamePieceType.kCone: {
            AngleType.kStow: {
                SubsystemType.kArm: 0,
                SubsystemType.kClaw: 0,
            },
            AngleType.kFloor: {
                SubsystemType.kArm: 45,
                SubsystemType.kClaw: 90,
            },
            AngleType.kUpperFeedStation: {
                SubsystemType.kArm: 120,
                SubsystemType.kClaw: 90,
            },
            AngleType.kGridS1: {
                SubsystemType.kArm: 60,
                SubsystemType.kClaw: 90,
            },
            AngleType.kGridS2: {
                SubsystemType.kArm: 84,
                SubsystemType.kClaw: 20,
            },
            AngleType.kGridS3: {
                SubsystemType.kArm: 100,
                SubsystemType.kClaw: 100,
            },
        },
        GamePieceType.kCube: {
            AngleType.kStow: {
                SubsystemType.kArm: 0,
                SubsystemType.kClaw: 0,
            },
            AngleType.kFloor: {
                SubsystemType.kArm: 45,
                SubsystemType.kClaw: 90,
            },
            AngleType.kUpperFeedStation: {
                SubsystemType.kArm: 120,
                SubsystemType.kClaw: 90,
            },
            AngleType.kGridS1: {
                SubsystemType.kArm: 60,
                SubsystemType.kClaw: 90,
            },
            AngleType.kGridS2: {
                SubsystemType.kArm: 80,
                SubsystemType.kClaw: 90,
            },
            AngleType.kGridS3: {
                SubsystemType.kArm: 100,
                SubsystemType.kClaw: 100,
            },
        },
        GamePieceType.kEmpty: {
            AngleType.kStow: {
                SubsystemType.kArm: 0,
                SubsystemType.kClaw: 0,
            },
            AngleType.kFloor: {
                SubsystemType.kArm: 45,
                SubsystemType.kClaw: 90,
            },
            AngleType.kUpperFeedStation: {
                SubsystemType.kArm: 120,
                SubsystemType.kClaw: 90,
            },
            AngleType.kGridS1: {
                SubsystemType.kArm: 60,
                SubsystemType.kClaw: 90,
            },
            AngleType.kGridS2: {
                SubsystemType.kArm: 80,
                SubsystemType.kClaw: 90,
            },
            AngleType.kGridS3: {
                SubsystemType.kArm: 100,
                SubsystemType.kClaw: 100,
            },
        },
    }

    usePreferences = True
    # ! this will override the values in angles
    if usePreferences:
        for gamePiece in angles:
            for angleType in angles[gamePiece]:
                for subsystemType in angles[gamePiece][angleType]:
                    preferencesAngle = wpilib.Preferences.getDouble(
                        f"{gamePiece}/{angleType}/{subsystemType}", -1
                    )
                    if preferencesAngle != -1:
                        angles[gamePiece][angleType][subsystemType] = preferencesAngle

    class Arm:
        motorType = rev.CANSparkMax.MotorType.kBrushed
        speedScale = 0.6
        currentLimit = 40  # amps

        initialPosition = 0.0  # degrees
        angleTolerance = 4.0  # degrees
        encoderOffsetHack = 360.0  # degrees

        kCANId = 10
        kConversionFactorToDeg = 27.6341
        kMaxVelocityRPM = 2000  # TODO calibrate
        kMaxAccelerationRPM = 1500  # TODO calibrate

        kForwardSoftLimit = 120  # degrees
        kReverseSoftLimit = 0  # degrees

        getEncoderArgs = (rev.SparkMaxRelativeEncoder.Type.kQuadrature, 1024)

        kP = 0.001  # TODO this may have to be non-zero for smart motion to work
        kI = 0
        kIz = 0
        kD = 0
        kFF = 0
        # TODO calibrate
        kS = 0.02
        kG = 1.0
        kV = 0.0
        kA = 0.0

        class Manual:
            maxAnglePerSecond = 10  # degrees per second
            omegaScale = 1

    class Claw:
        motorType = rev.CANSparkMax.MotorType.kBrushless
        speedScale = 0.2
        currentLimit = 20  # amps

        initialPosition = 180.0  # degrees
        angleTolerance = 4.0  # degrees
        # Hack to stop the encoder from going below 0 and underflowing
        encoderOffsetHack = 0.0  # degrees

        kCANId = 11
        kConversionFactorToDeg = 2.99089
        kMaxVelocityRPM = 1500  # TODO calibrate
        kMaxAccelerationRPM = 500  # TODO calibrate

        kForwardSoftLimit = 180  # degrees
        kReverseSoftLimit = 0  # degrees

        getEncoderArgs = ()

        kP = 0.05
        kI = 0
        kIz = 0
        kD = 0
        kFF = 0
        # TODO calibrate
        kS = 0
        kG = -0.5
        kV = 0.0
        kA = 0.0

        class Manual:
            maxAnglePerSecond = 10  # degrees per second
            omegaScale = 1

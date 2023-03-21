from enum import Enum
import math
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d


class FalconConstants:
    kMotorFreeSpeedRPM = 6380
    kUnitsPerRotation = 2048


class Constants:
    # !
    max_speed = 0.15
    scale_speed = 0.15

    period = 0.02

    controller_deadzone = 0.05
    pilot_controller_id = 0
    operator_controller_id = 1

    frame_width = 28
    frame_length = 32

    light_strip_pwm_port = 1


class ArmConstants:
    class GamePieceType(Enum):
        kCone = 0
        kCube = 1
        kEmpty = 2

    class AngleType(Enum):
        kStow = 0
        kFloor = 1
        kGridS1 = 2
        kGridS2 = 3
        kGridS3 = 4

    class SubsystemType(Enum):
        kArm = 0
        kClaw = 1

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
        GamePieceType.kCube: {
            AngleType.kStow: {
                SubsystemType.kArm: 0,
                SubsystemType.kClaw: 0,
            },
            AngleType.kFloor: {
                SubsystemType.kArm: 45,
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

    class Arm:
        kCANId = 10
        kConversionFactor = 0.5532  # TODO change
        kMaxVelocityRPM = 2000  # TODO calibrate
        kMaxAccelerationRPM = 1500  # TODO calibrate

        kP = 0  # TODO this may have to be non-zero for smart motion to work
        kI = 0
        kIz = 0
        kD = 0
        kFF = 0
        # TODO calibrate
        kS = 0.05
        kG = 3.34
        kV = 0.47
        kA = 0.55

    class Claw:
        kCANId = 11
        kConversionFactor = 10  # TODO change
        kMaxVelocityRPM = 1000  # TODO calibrate
        kMaxAccelerationRPM = 500  # TODO calibrate

        kP = 0
        kI = 0
        kIz = 0
        kD = 0
        kFF = 0
        # TODO calibrate
        kS = 0
        kG = 9.04
        kV = 0.10
        kA = 0.23


class SwerveConstants:
    swerveDashboardName = "Swerve Dashboard"

    # to change the robot orientation (which way is front), change these
    # kDriveKinematics = SwerveDrive4Kinematics(
    #     Translation2d(-Constants.frame_width / 2, -Constants.frame_length / 2),
    #     Translation2d(Constants.frame_width / 2, -Constants.frame_length / 2),
    #     Translation2d(Constants.frame_width / 2, Constants.frame_length / 2),
    #     Translation2d(-Constants.frame_width / 2, Constants.frame_length / 2),
    # )
    kDriveKinematics = SwerveDrive4Kinematics(
        Translation2d(Constants.frame_width / 2, -Constants.frame_length / 2),
        Translation2d(Constants.frame_width / 2, Constants.frame_length / 2),
        Translation2d(-Constants.frame_width / 2, -Constants.frame_length / 2),
        Translation2d(-Constants.frame_width / 2, Constants.frame_length / 2),
    )

    fl_drive_id = 2
    fl_turn_id = 3
    fl_abs_encoder_offset_rad = 2.6546849
    # fl_chassis_angular_offset = 0
    fl_chassis_angular_offset = 0

    fr_drive_id = 8
    fr_turn_id = 9
    fr_abs_encoder_offset_rad = 0.7045867
    # fr_chassis_angular_offset = math.pi / 2
    fr_chassis_angular_offset = -math.pi / 2

    bl_drive_id = 6
    bl_turn_id = 7
    bl_abs_encoder_offset_rad = 5.2724060
    # bl_chassis_angular_offset = -math.pi / 2
    bl_chassis_angular_offset = math.pi

    br_drive_id = 4
    br_turn_id = 5
    br_abs_encoder_offset_rad = 0.5696462
    # br_chassis_angular_offset = math.pi
    br_chassis_angular_offset = math.pi / 2

    kPTurning = 1
    # kPTurning = 0.4
    kITurning = 0
    kDTurning = 0

    # TODO calibrate
    kDriveMaxMetersPerSecond = 5.15
    kDriveMaxAccelerationMetersPerSecond = 3.0
    kDriveMaxTurnAccelerationMetersPerSecond = 5.0

    kPRobotTurn = 0.001
    kIRobotTurn = 0
    kDRobotTurn = 0

    inches_to_meters = 39.37

    kDrivingMotorReduction = 5.08

    kWheelDiameterMeters = 3 / inches_to_meters
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    # kWheelRotationsPerMeter = 1 / kWheelCircumference
    kEncoderPulsesPerRevolution = (
        kDrivingMotorReduction * FalconConstants.kUnitsPerRotation
    )  # manually measured as 10532
    kEncoderPositionPerMeter = kEncoderPulsesPerRevolution / kWheelCircumferenceMeters

    # print("kEncoderPositionPerMeter", kEncoderPositionPerMeter)

    kDriveWheelFreeSpeedRps = (
        FalconConstants.kMotorFreeSpeedRPM * kWheelCircumferenceMeters
    ) / kDrivingMotorReduction

    # print("kDriveWheelFreeSpeedRps", kDriveWheelFreeSpeedRps)

    # kDrivingEncoderPositionFactor = (
    #     kWheelCircumferenceMeters
    # ) / kDrivingMotorReduction  # meters per encoder position

    # kDrivingEncoderPositionFactor = (
    #     kWheelCircumferenceMeters * kDrivingMotorReduction
    # ) / ((FalconConstants.kMotorFreeSpeedRPM / 60) * kEncoderPulsesPerRevolution)

    # kDrivingEncoderVelocityFactor = (
    #     kWheelCircumferenceMeters / kDrivingMotorReduction
    # ) / 60.0  # meters per second
    # # kDrivingEncoderVelocityFactor = 3.135

    # calibrated_vs = (vs / kEncoderPositionPerMeter) * 10 # multiplied by 10 to get m/100ms to m/s
    # vs = (calibrated_vs / 10) * kEncoderPositionPerMeter

    # TODO tune this once it is on the ground
    kMaxMeasuredVelocityEncoderUnits = 18000
    typical_drive_velocity = 0.8
    kFDriving = (typical_drive_velocity * 1023) / kMaxMeasuredVelocityEncoderUnits

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
    copilot_controller_id = 1

    frame_width = 28
    frame_length = 32

    robot_angle_offset = math.pi / 2

    light_strip_pwm_port = 0


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
    kDriveMaxMetersPerSecond = 4.0
    kDriveMaxAccelerationMetersPerSecond = 3.0
    kDriveMaxTurnAccelerationMetersPerSecond = 5.0

    kPRobotTurn = 0.05
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

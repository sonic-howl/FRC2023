class Constants:
    # !
    max_speed = 1
    scale_speed = 0.15

    period = 0.02

    controller_deadzone = 0.05
    pilot_controller_id = 0
    copilot_controller_id = 0

    frame_width = 28
    frame_length = 32


import math
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d


class SwerveConstants:
    kDriveKinematics = SwerveDrive4Kinematics(
        Translation2d(-Constants.frame_width / 2, Constants.frame_length / 2),
        Translation2d(Constants.frame_width / 2, Constants.frame_length / 2),
        Translation2d(Constants.frame_width / 2, -Constants.frame_length / 2),
        Translation2d(-Constants.frame_width / 2, -Constants.frame_length / 2),
    )

    fr_drive_id = 2
    fr_turn_id = 3
    fr_abs_encoder_offset_rad = 2.6546849
    fr_chassis_angular_offset = math.pi / 2

    br_drive_id = 4
    br_turn_id = 5
    br_abs_encoder_offset_rad = 0.5696462
    br_chassis_angular_offset = math.pi

    bl_drive_id = 6
    bl_turn_id = 7
    bl_abs_encoder_offset_rad = 5.2724060
    bl_chassis_angular_offset = math.pi / 2

    fl_drive_id = 8
    fl_turn_id = 9
    fl_abs_encoder_offset_rad = 0.7045867
    fl_chassis_angular_offset = 0

    kPTurning = 0.18
    kITurning = 0
    kDTurning = 0
    kTurningToleranceDeg = 1.5

    # TODO calibrate
    kDriveMaxMetersPerSecond = 5.0
    kDriveMaxAccelerationMetersPerSecond = 10.0
    kDriveMaxTurnAccelerationMetersPerSecond = 5.0

    kPRobotTurn = 0.05
    kIRobotTurn = 0
    kDRobotTurn = 0

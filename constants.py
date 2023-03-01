class Constants:
    period = 0.02

    controller_deadzone = 0.05
    pilot_controller_id = 0
    copilot_controller_id = 0

    frame_width = 28
    frame_length = 32


from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d


class SwerveConstants:
    kDriveKinematics = SwerveDrive4Kinematics(
        Translation2d(-Constants.frame_width / 2, Constants.frame_length / 2),
        Translation2d(Constants.frame_width / 2, Constants.frame_length / 2),
        Translation2d(Constants.frame_width / 2, -Constants.frame_length / 2),
        Translation2d(-Constants.frame_width / 2, -Constants.frame_length / 2),
    )

    fl_drive_id = 2
    fl_turn_id = 3
    fl_abs_encoder_id = 0

    fr_drive_id = 4
    fr_turn_id = 5
    fr_abs_encoder_id = 1

    bl_drive_id = 6
    bl_turn_id = 7
    bl_abs_encoder_id = 2

    br_drive_id = 8
    br_turn_id = 9
    br_abs_encoder_id = 3

    kPTurning = 0.5
    kITurning = 0
    kDTurning = 0

    kDriveMaxMetersPerSecond = 5.0
    kDriveMaxAccelerationMetersPerSecond = 10.0
    kDriveMaxTurnAccelerationMetersPerSecond = 5.0

    kPRobotTurn = 0.05
    kIRobotTurn = 0
    kDRobotTurn = 0

    kWheelMaxSpeedMetersPerSecond = 5.0

    kDriveEncoderRotToMeters = 4.0
    kDriveEncoderRotToVelocityMps = 0.5
    kTurnEncoderRotToMeters = 0.5
    kTurnEncoderRotToVelocityMps = 0.25

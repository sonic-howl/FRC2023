from typing import List
from LightStrip import LightStrip

from photonvision import PhotonCamera, LEDMode, RobotPoseEstimator
from wpilib import Field2d, SmartDashboard
from commands.SwerveAutoCommand import SwerveAutoCommand
from pathplannerlib import PathPlanner, PathConstraints, PathPlannerTrajectory
from pathplannerlib._pathplannerlib.controllers import PPHolonomicDriveController
from utils import print_async, sign
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from wpimath.trajectory import (
    TrajectoryConfig,
    TrajectoryGenerator,
    TrapezoidProfileRadians,
)
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.controller import (
    PIDController,
    ProfiledPIDControllerRadians,
)
from commands2 import (
    RunCommand,
    InstantCommand,
    Swerve4ControllerCommand,
    SequentialCommandGroup,
    ParallelCommandGroup,
)
from commands2.button import JoystickButton, CommandXboxController

from subsystems.SwerveSubsystem import SwerveSubsystem
from commands.SwerveCommand import SwerveCommand
from constants import Constants, SwerveConstants


from wpimath.filter._filter import SlewRateLimiter


def dz(x: float, dz: float = Constants.controller_deadzone):
    return x if abs(x) > dz else 0


class RobotContainer:
    swerve_subsystem = SwerveSubsystem()

    controller = CommandXboxController(Constants.pilot_controller_id)

    field_oriented = False

    right_stick_sets_angle = False

    photon_camera = PhotonCamera("photonvision")

    def __init__(self) -> None:
        self.configure_button_bindings()

        self.swerve_subsystem.setDefaultCommand(
            SwerveCommand(
                self.swerve_subsystem,
                self.controller,
                self.get_right_stick_sets_angle,
                self.get_field_oriented,
            )
        )

        self.x_limiter = SlewRateLimiter(
            SwerveConstants.kDriveMaxAccelerationMetersPerSecond
        )
        self.y_limiter = SlewRateLimiter(
            SwerveConstants.kDriveMaxAccelerationMetersPerSecond
        )
        self.z_limiter = SlewRateLimiter(
            SwerveConstants.kDriveMaxTurnAccelerationMetersPerSecond
        )

        self.rotate_to_angle_pid = PIDController(
            SwerveConstants.kPRobotTurn,
            SwerveConstants.kIRobotTurn,
            SwerveConstants.kDRobotTurn,
            period=Constants.period,
        )
        self.rotate_to_angle_pid.enableContinuousInput(-180, 180)
        self.rotate_to_angle_pid.setTolerance(3)  # degrees tolerance

        self.swerve_auto_command = SwerveAutoCommand(self.swerve_subsystem)

        self.robot_angle_offset = Rotation2d(Constants.robot_angle_offset)

        self.light_strip = LightStrip(Constants.light_strip_pwm_port)
        self.light_strip.setRainbowSlow()

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

    def get_right_stick_sets_angle(self) -> bool:
        return self.right_stick_sets_angle

    def get_angle(self):
        return self.swerve_subsystem.get_angle()

    def vision_track(self):
        if self.photon_camera.hasTargets():
            target = self.photon_camera.getLatestResult().getBestTarget()
            transform_to_target = target.getBestCameraToTarget()
            # TODO

    def robotPeriodic(self):
        self.light_strip.update()

        self.swerve_subsystem.periodic()
        self.field.setRobotPose(self.swerve_subsystem.get_pose())

        if self.controller.isConnected():
            if self.controller.getAButton():
                self.vision_track()

    def teleopPeriodic(self) -> None:
        if not self.controller.isConnected():
            self.swerve_subsystem.stop()
            return

        # if this is made into a command it would make more sense
        if self.controller.getXButton():
            self.swerve_subsystem.setX()
            return

        if self.controller.getBButton():
            self.swerve_subsystem.reset_motor_positions()
            self.swerve_subsystem.reset_odometer()

        if self.controller.getYButtonPressed():
            self.toggle_field_oriented()

        # # reversing x and y controller -> field axes. x is forwards, y is strafe
        # # normally axis 0 is x.
        # x = dz(-self.controller.getRawAxis(1))
        # y = dz(-self.controller.getRawAxis(0))

        # # x = self.x_limiter.calculate(x)
        # # y = self.y_limiter.calculate(y)

        # chassis_speeds: ChassisSpeeds | None = None
        # if self.get_right_stick_sets_angle():
        #     rx = dz(self.controller.getRawAxis(4))
        #     ry = dz(self.controller.getRawAxis(5))

        #     magnitude = abs(rx) + abs(ry)

        #     if magnitude > 0.25:
        #         rotation2d = Rotation2d(rx, ry)

        #         current_robot_angle = self.swerve_subsystem.get_angle()

        #         print(current_robot_angle, rotation2d.degrees())

        #         rotation_speed = self.rotate_to_angle_pid.calculate(
        #             current_robot_angle, rotation2d.degrees()
        #         )

        #         chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        #             x * SwerveConstants.kDriveMaxMetersPerSecond,
        #             y * SwerveConstants.kDriveMaxMetersPerSecond,
        #             rotation_speed,
        #             self.swerve_subsystem.get_rotation2d(),
        #         )

        #     # print("SwerveCommand chassis_speeds: ", chassis_speeds)
        # elif self.get_field_oriented():
        #     # z = dz(self.controller.getRightX())
        #     z = self.controller.getRawAxis(4)
        #     z = (dz(z) ** 2) * sign(z)
        #     z = self.z_limiter.calculate(z)
        #     chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        #         x * SwerveConstants.kDriveMaxMetersPerSecond,
        #         y * SwerveConstants.kDriveMaxMetersPerSecond,
        #         z,
        #         self.swerve_subsystem.get_rotation2d(),
        #     )
        # else:
        #     z = -self.controller.getRawAxis(4)
        #     z = (dz(z) ** 2) * sign(z)
        #     magnitude = abs(x) + abs(y) + abs(z)
        #     if magnitude > 0.05:
        #         # z = self.z_limiter.calculate(z) # TODO
        #         # chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        #         #     x * SwerveConstants.kDriveMaxMetersPerSecond,
        #         #     y * SwerveConstants.kDriveMaxMetersPerSecond,
        #         #     z,
        #         #     self.robot_angle_offset,
        #         # )
        #         chassis_speeds = ChassisSpeeds(
        #             x * SwerveConstants.kDriveMaxMetersPerSecond,
        #             y * SwerveConstants.kDriveMaxMetersPerSecond,
        #             z,
        #         )

        # if chassis_speeds:
        #     swerve_module_states = (
        #         SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassis_speeds)
        #     )

        #     # if (
        #     #     abs(chassis_speeds.vx) > 0
        #     #     or abs(chassis_speeds.vy) > 0
        #     #     or abs(chassis_speeds.omega) > 0
        #     # ):
        #     #     print_async(
        #     #         chassis_speeds,
        #     #         # "SwerveCommand swerve_module_states: ",
        #     #         # swerve_module_states,
        #     #     )
        #     # print_async(self.swerve_subsystem.get_pose())

        #     self.swerve_subsystem.set_module_states(swerve_module_states)
        # else:
        #     self.swerve_subsystem.stop()

    def autonomousInit(self):
        self.swerve_auto_command.initialize()

    def autonomousPeriodic(self) -> None:
        if self.swerve_auto_command.isFinished():
            self.swerve_subsystem.stop()
        else:
            self.swerve_auto_command.execute()

    def get_field_oriented(self) -> bool:
        return self.field_oriented

    def toggle_field_oriented(self) -> None:
        self.field_oriented = not self.field_oriented
        print("Field oriented: ", self.field_oriented)

    def configure_button_bindings(self) -> None:
        if self.controller.isConnected():
            self.controller.X().onTrue(InstantCommand(self.toggle_field_oriented))

    def path_group_to_poses(
        self, path_group: List[PathPlannerTrajectory]
    ) -> list[Pose2d]:
        poses = []
        # all_markers = []
        for trajectory in path_group:
            # TODO use these
            events = trajectory.getMarkers()
            stop_event = trajectory.getEndStopEvent()
            for state in trajectory.getStates():
                poses.append(state.pose)

        return poses

    def buildPPAutonomous(self):
        path_group = PathPlanner.loadPathGroup(
            "FullPath",
            SwerveConstants.kDriveMaxMetersPerSecond,
            SwerveConstants.kDriveMaxAccelerationMetersPerSecond,
        )

        x_pid = PIDController(0.5, 0, 0, period=Constants.period)
        y_pid = PIDController(0.5, 0, 0, period=Constants.period)
        theta_pid = ProfiledPIDControllerRadians(
            0.5,
            0,
            0,
            TrapezoidProfileRadians.Constraints(3, 3),
            period=Constants.period,
        )

        # for trajectory in path_group:
        #     for state in trajectory.getStates():
        #         chassis_speeds = controller.calculate(
        #             self.swerve_subsystem.get_pose(), state
        #         )
        #         swerve_module_states = (
        #             SwerveConstants.kDriveKinematics.toSwerveModuleStates(
        #                 chassis_speeds
        #             )
        #         )
        #         self.swerve_subsystem.set_module_states(swerve_module_states)

        # poses = self.path_group_to_poses(path_group)

        # trajectory_config = TrajectoryConfig(
        #     SwerveConstants.kDriveMaxMetersPerSecond,
        #     SwerveConstants.kDriveMaxAccelerationMetersPerSecond,
        # )
        # trajectory_config.setKinematics(SwerveConstants.kDriveKinematics)

        # trajectory = TrajectoryGenerator.generateTrajectory(
        #     poses,
        #     trajectory_config,
        # )

    def getAutonomousCommand(self):
        trajectory_config = TrajectoryConfig(
            SwerveConstants.kDriveMaxMetersPerSecond,
            SwerveConstants.kDriveMaxAccelerationMetersPerSecond,
        )
        trajectory_config.setKinematics(SwerveConstants.kDriveKinematics)

        trajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(0, 0, Rotation2d(0)),
            [Translation2d(1, 1), Translation2d(2, -1)],
            Pose2d(3, 0, Rotation2d(0)),
            trajectory_config,
        )

        # TODO make these constants in constants.py
        x_pid = PIDController(0.5, 0, 0, period=Constants.period)
        y_pid = PIDController(0.5, 0, 0, period=Constants.period)
        theta_pid = ProfiledPIDControllerRadians(
            0.5,
            0,
            0,
            TrapezoidProfileRadians.Constraints(3, 3),
            period=Constants.period,
        )
        theta_pid.enableContinuousInput(-180, 180)

        swerve_command = Swerve4ControllerCommand(
            trajectory,
            self.swerve_subsystem.get_pose,
            SwerveConstants.kDriveKinematics,
            x_pid,
            y_pid,
            theta_pid,
            self.swerve_subsystem.set_module_states_list,
            [self.swerve_subsystem],
        )

        return SequentialCommandGroup(
            InstantCommand(
                self.swerve_subsystem.reset_odometer, trajectory.initialPose
            ),
            swerve_command,
            InstantCommand(self.swerve_subsystem.stop),
        )

import typing

from commands2 import Command, Subsystem
from pathplannerlib import PathPlannerTrajectory
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.trajectory import TrapezoidProfileRadians

from commands.Claw.StowCommand import StowCommand
from constants.ArmConstants import ArmConstants
from constants.GameConstants import GamePieceType
from constants.RobotConstants import RobotConstants
from constants.SwerveConstants import SwerveConstants
from subsystems.Arm.ArmAssemblySubsystem import ArmAssemblySubsystem
from subsystems.Swerve.SwerveSubsystem import SwerveSubsystem
from utils.utils import printAsync

from .PathPlanner import PathTraverser


class SwerveAutoCommand(Command):
    def __init__(
        self,
        swerveSubsystem: SwerveSubsystem,
        armAssemblySubsystem: ArmAssemblySubsystem,
    ) -> None:
        super().__init__()

        self.swerveSubsystem = swerveSubsystem
        self.armAssemblySubsystem = armAssemblySubsystem

        # TODO maybe move these to the swerve class, or have something similar
        x_pid = PIDController(1, 0, 0, period=RobotConstants.period)
        y_pid = PIDController(1, 0, 0, period=RobotConstants.period)
        theta_pid = ProfiledPIDControllerRadians(
            0.5,
            0,
            0,
            TrapezoidProfileRadians.Constraints(
                SwerveConstants.kDriveMaxTurnMetersPerSecond,
                SwerveConstants.kDriveMaxTurnAccelerationMetersPerSecond,
            ),
            period=RobotConstants.period,
        )
        self.controller = HolonomicDriveController(x_pid, y_pid, theta_pid)
        self.controller.setTolerance(Pose2d(0.05, 0.05, Rotation2d.fromDegrees(2)))
        # self.controller = PPHolonomicDriveController(x_pid, y_pid, theta_pid)
        # theta_pid = PIDController(0, 0, 0, period=Constants.period)

        # self.traverser = PathTraverser("Forwards_1m")
        # self.traverser = PathTraverser("Spin_90")
        # self.traverser = PathTraverser("Forwards_1m Spin_90")
        self.traverser = PathTraverser("FullPath")

        def handle_stop(stop_event: PathPlannerTrajectory.StopEvent) -> None:
            printAsync(
                "STOP EVENT",
                stop_event.waitBehavior,
                stop_event.names,
                "\nwait time:",
                stop_event.waitTime,
                "\nFinal Pose:",
                self.swerveSubsystem.getPose(),
            )

        self.traverser.on_stop(handle_stop)
        self.traverser.on(
            f"{GamePieceType.kCone.name}/{ArmConstants.AngleType.kStow.name}",
            StowCommand(self.armAssemblySubsystem),
        )

        self.finished = False

    def getRequirements(self) -> typing.Set[Subsystem]:
        return {self.swerveSubsystem}

    def initialize(self) -> None:
        self.finished = False
        self.swerveSubsystem.resetOdometer()
        self.swerveSubsystem.resetGyro()
        self.traverser.reset()

        state = self.traverser.get_initial_state()
        self.swerveSubsystem.swerveAutoStartPose = state.pose
        self.swerveSubsystem.resetOdometer(state.pose)  # may be problematic
        # self.move_to_state(state)
        # print_async(
        #     "SwerveAutoCommand initialized:",
        #     "setpoint",
        #     state.pose,
        #     "current",
        #     self.swerve_subsystem.get_pose(),
        # )

    # def move_to_state(self, state: Trajectory.State):

    def move_to_state(self, state: PathPlannerTrajectory.PathPlannerState):
        chassisSpeeds = self.controller.calculate(
            self.swerveSubsystem.getPose(),
            state.pose,
            state.velocity,
            state.holonomicRotation,
        )

        # chassis_speeds = self.controller.calculate(
        #     self.swerve_subsystem.get_pose(), state, state.pose.rotation()
        # )
        # print(
        #     "current_pose:  ",
        #     self.swerveSubsystem.getPose(),
        #     "\nsetpoint rot:  ",
        #     state.holonomicRotation.degrees(),
        #     "\nchassis_speeds:",
        #     chassisSpeeds,
        # )

        #         print(
        #             f"""
        # current_pose:   {self.swerveSubsystem.getPose()}
        # setpoint rot:   {state.holonomicRotation.degrees()}
        # chassis_speeds: {chassisSpeeds}"""
        #         )

        if RobotConstants.isSimulation:
            self.swerveSubsystem.simChassisSpeeds = chassisSpeeds

        swerveModuleStates = SwerveSubsystem.toSwerveModuleStatesForecast(chassisSpeeds)

        self.swerveSubsystem.setModuleStates(swerveModuleStates, isClosedLoop=True)

        # print_async(self.traverser.timer.get(), swerve_module_states)

    def execute(self) -> None:
        self.traverser.start_timer()
        state = self.traverser.sample()

        # if state is None:
        #     if self.controller.atReference():
        #         self.end(False)
        #     return

        self.move_to_state(state)

    def end(self, interrupted: bool) -> None:
        self.swerveSubsystem.stop()

        if not interrupted:
            printAsync(
                "SwerveAutoCommand ended",
                "iterations:",
                self.traverser.iterations,
                "total time:",
                self.traverser.total_time,
            )
            self.finished = True
        else:
            print("SwerveAutoCommand interrupted")

    def isFinished(self) -> bool:
        return self.finished

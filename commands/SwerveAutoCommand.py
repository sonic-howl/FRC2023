import math
from time import sleep
from typing import Callable, Dict, List
import typing

from commands2 import Command, Subsystem
from physics import PhysicsEngine
from utils.utils import printAsync

import wpilib
from constants import Constants, SwerveConstants
from pathplannerlib import PathPlanner, PathPlannerTrajectory

# from pathplannerlib._pathplannerlib.controllers import PPHolonomicDriveController
from subsystems.Swerve.SwerveSubsystem import SwerveSubsystem
from wpimath.controller import (
    PIDController,
    ProfiledPIDControllerRadians,
    HolonomicDriveController,
)

from wpimath.geometry import Transform2d, Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrapezoidProfileRadians, Trajectory


class PathTraverser:
    event_callbacks: Dict[str, Callable[[PathPlannerTrajectory.EventMarker], None]] = {}
    stop_event_callbacks: List[Callable[[PathPlannerTrajectory.StopEvent], None]] = []

    event_markers: List[PathPlannerTrajectory.EventMarker] = []

    def __init__(self, name: str = "FullPath") -> None:
        path_group = PathPlanner.loadPathGroup(
            name,
            SwerveConstants.kDriveMaxMetersPerSecond,
            SwerveConstants.kDriveMaxAccelerationMetersPerSecond,
        )

        print(f"Loaded path group: {name}")
        print("Start pose:", path_group[0].getInitialPose())
        print("End pose:  ", path_group[-1].getEndState().pose)

        self.path_group = path_group

        self.reset()

    def reset(self) -> None:
        self.current_path_index = 0
        self.current_path = self.path_group[self.current_path_index]
        self.stop_event = self.current_path.getEndStopEvent()
        self.current_path_initial_state = self.get_initial_state()

        self.timer = wpilib.Timer()

        self.paths_traversed = False

        self.total_time = 0
        self.iterations = 0

    def start_timer(self) -> None:
        self.timer.start()

    def get_initial_state(self):
        return self.current_path.getInitialState().asWPILibState()

    def get_new_event_marker(self):
        if len(self.event_markers) == 0:
            return None
        event_marker = self.event_markers[0]
        if event_marker.time <= self.timer.get():
            self.event_markers.pop(0)
            return event_marker
        return None

    def move_to_next_path(self) -> None:
        if self.current_path_index + 1 >= len(self.path_group):
            self.paths_traversed = True
            return
        self.current_path_index += 1
        self.current_path = self.path_group[self.current_path_index]
        self.total_time += self.timer.get()
        self.timer.reset()
        self.timer.start()

        self.event_markers = self.current_path.getMarkers()
        self.stop_event = self.current_path.getEndStopEvent()

        # def sample(self) -> Trajectory.State | None:

    # def sample(self) -> PathPlannerTrajectory.PathPlannerState | None:
    def sample(self) -> PathPlannerTrajectory.PathPlannerState:
        # if self.paths_traversed:
        #     return None

        total_time = self.current_path.getTotalTime()
        time = self.timer.get()
        event_marker = self.get_new_event_marker()
        if event_marker:
            for name in event_marker.names:
                # possible bug here when there are events with the same name at different times.
                # make sure every name is unique
                cb = self.event_callbacks.get(name)
                if cb:
                    cb(event_marker)
                    del self.event_callbacks[name]
        if time > total_time:
            for stop_callback in self.stop_event_callbacks:
                stop_callback(self.stop_event)
                self.stop_event_callbacks.remove(stop_callback)
            self.move_to_next_path()

        self.iterations += 1

        return self.current_path.sample(time)

    def on(
        self, name: str, func: Callable[[PathPlannerTrajectory.EventMarker], None]
    ) -> None:
        self.event_callbacks[name] = func

    def on_stop(self, func: Callable[[PathPlannerTrajectory.StopEvent], None]) -> None:
        self.stop_event_callbacks.append(func)


class SwerveAutoCommand(Command):
    def __init__(self, swerve_subsystem: SwerveSubsystem) -> None:
        super().__init__()

        self.swerveSubsystem = swerve_subsystem

        # TODO
        x_pid = PIDController(1, 0, 0, period=Constants.period)
        y_pid = PIDController(1, 0, 0, period=Constants.period)
        theta_pid = ProfiledPIDControllerRadians(
            5,
            0,
            0,
            # TrapezoidProfileRadians.Constraints(),
            TrapezoidProfileRadians.Constraints(
                SwerveConstants.kDriveMaxAccelerationMetersPerSecond,
                SwerveConstants.kDriveMaxTurnAccelerationMetersPerSecond,
            ),
            period=Constants.period,
        )
        self.controller = HolonomicDriveController(x_pid, y_pid, theta_pid)
        self.controller.setTolerance(Pose2d(0.05, 0.05, Rotation2d.fromDegrees(2)))
        # self.controller = PPHolonomicDriveController(x_pid, y_pid, theta_pid)
        # theta_pid = PIDController(0, 0, 0, period=Constants.period)

        # self.traverser = PathTraverser("Forwards_1m")
        # self.traverser = PathTraverser("Spin_90")
        self.traverser = PathTraverser("Forwards_1m Spin_90")

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

        print(
            f"""
current_pose:   {self.swerveSubsystem.getPose()}
setpoint rot:   {state.holonomicRotation.degrees()}
chassis_speeds: {chassisSpeeds}"""
        )

        if Constants.isSimulation:
            self.swerveSubsystem.chassisSpeeds = chassisSpeeds

        swerveModuleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(
            chassisSpeeds
        )

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

        if Constants.isSimulation:
            self.swerveSubsystem.chassisSpeeds = None

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

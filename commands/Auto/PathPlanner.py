from typing import Callable, Dict, List

import wpilib
from commands2 import Command
from pathplannerlib import PathPlanner, PathPlannerTrajectory

from constants.SwerveConstants import SwerveConstants


class PathTraverser:
    event_callbacks: Dict[
        str, Callable[[PathPlannerTrajectory.EventMarker], None] | Command
    ] = {}
    stop_event_callbacks: List[
        Callable[[PathPlannerTrajectory.StopEvent], None] | Command
    ] = []

    event_markers: List[PathPlannerTrajectory.EventMarker] = []

    def __init__(self, name: str) -> None:
        self.path_group = PathPlanner.loadPathGroup(
            name,
            SwerveConstants.kDriveMaxMetersPerSecond,
            SwerveConstants.kDriveMaxAccelerationMetersPerSecond,
        )
        self.transformedForBlueAlliance = True

        print(f"Loaded path group: {name}")
        print("Start pose:", self.path_group[0].getInitialPose())
        print("End pose:  ", self.path_group[-1].getEndState().pose)

        self.reset()

    def _transformTrajectoriesForAlliance(
        self, alliance: wpilib.DriverStation.Alliance
    ) -> None:
        for i in range(len(self.path_group)):
            self.path_group[i] = PathPlannerTrajectory.transformTrajectoryForAlliance(
                self.path_group[i], alliance
            )

    def transformTrajectoriesForAlliance(self) -> None:
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            if not self.transformedForBlueAlliance:
                self._transformTrajectoriesForAlliance(
                    wpilib.DriverStation.Alliance.kBlue
                )
                self.transformedForBlueAlliance = True
        else:
            if self.transformedForBlueAlliance:
                self._transformTrajectoriesForAlliance(
                    wpilib.DriverStation.Alliance.kRed
                )
                self.transformedForBlueAlliance = False

    def reset(self) -> None:
        self.transformTrajectoriesForAlliance()

        self.current_path_index = 0
        self.current_path = self.path_group[self.current_path_index]
        self.event_markers = self.current_path.getMarkers()
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
                    if isinstance(cb, Command):
                        cb.schedule()
                    else:
                        cb(event_marker)
        if time > total_time:
            for stop_cb in self.stop_event_callbacks:
                if isinstance(stop_cb, Command):
                    stop_cb.schedule()
                else:
                    stop_cb(self.stop_event)
                self.stop_event_callbacks.remove(stop_cb)
            self.move_to_next_path()

        self.iterations += 1

        return self.current_path.sample(time)

    def on(
        self,
        name: str,
        func: Callable[[PathPlannerTrajectory.EventMarker], None] | Command,
    ) -> None:
        self.event_callbacks[name] = func

    def on_stop(self, func: Callable[[PathPlannerTrajectory.StopEvent], None]) -> None:
        self.stop_event_callbacks.append(func)

import math
from ntcore import NetworkTableInstance
from wpilib import SmartDashboard
from controllers.operator import OperatorController
from commands2 import Command, InstantCommand, SequentialCommandGroup, Subsystem, Swerve4ControllerCommand
from typing import Callable, Set
from subsystems.Swerve.SwerveSubsystem import SwerveSubsystem
from constants import Constants, LimelightConstants, SwerveConstants
from wpimath.trajectory import Trajectory

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


class VisionTrackCommand(Command):
    
    def __init__(self, swerveSubsystem: SwerveSubsystem) -> None:
        super().__init__()
        self.ntInstance = NetworkTableInstance.getDefault()
        self.limelightTable = self.ntInstance.getTable("limelight")
        self.swerveSubsystem = swerveSubsystem
    
    def getRequirements(self) -> Set[Subsystem]:
        return {self.swerveSubsystem}

    def initialize(self) -> None:
        print("VisionTrack initialized")
        # Switch to reflective tape detection pipeline
        self.limelightTable.getEntry("pipeline").setInteger(LimelightConstants.reflectiveTapePipelineID) # Not sure about these constant values

    def execute(self) -> None:
        # Get robot's current pose
        robotPose = self.swerveSubsystem.odometer.getEstimatedPosition()
        # Get pose of target in robot space
        self.targetPoseRobotSpaceArray = self.limelightTable.getEntry('t6t_rs').getDoubleArray([0,0,0])

        targetPoseRobotSpace = Pose2d(self.targetPoseRobotSpaceArray[0], self.targetPoseRobotSpaceArray[1], self.targetPoseRobotSpaceArray[2])

        # Returns the targetPose relative to the Robot's Pose
        targetPoseRelative = targetPoseRobotSpace.relativeTo(robotPose)

        SmartDashboard.putNumber("Robot Pose x:", robotPose.x)
        SmartDashboard.putNumber("Robot Pose y:", robotPose.y)
        SmartDashboard.putNumber("Robot Pose Rotation:", robotPose.rotation().degrees())

        SmartDashboard.putNumber("targetPoseRelative x:", targetPoseRelative.x)
        SmartDashboard.putNumber("targetPoseRelative y:", targetPoseRelative.y)
        SmartDashboard.putNumber("targetPoseRelative Rotation:", targetPoseRelative.rotation().degrees())

        # Add pose of robot to target to get the pose of the target relavite to the field
        #targetPoseRelativeToRobot = targetPoseRobotSpace.relativeTo(robotPose) # Note sure if you can add poses. I want to add them like vectors
        
        # This uses the Trajectory class
        currentVelocity = math.sqrt(math.pow(self.swerveSubsystem.currentChassisSpeeds.vx,2) +  math.pow(self.swerveSubsystem.currentChassisSpeeds.vy,2))
        startState = Trajectory.State(0,currentVelocity,0,robotPose)    # Need to find a way to calculate current acceleration. For now it's assumed to be 0
        endState = Trajectory.State(4,0,0,targetPoseRelative)
        middleState = startState.interpolate(endState, 1/4)
        states = [startState, middleState, endState]
        print(states)
        trajectory = Trajectory(states)

        """ This uses the TrajectoryGenerator class
        trajectory_config = TrajectoryConfig(
        SwerveConstants.kDriveMaxMetersPerSecond,
        SwerveConstants.kDriveMaxAccelerationMetersPerSecond,
        )
        trajectory_config.setKinematics(SwerveConstants.kDriveKinematics)
        
        trajectory = TrajectoryGenerator.generateTrajectory(
            robotPose,
            [Translation2d(targetPoseRelative.x /2, targetPoseRelative.y /2)],
            targetPoseRelative,
            trajectory_config,
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
        theta_pid.enableContinuousInput(-180, 180)

        swerve_command = Swerve4ControllerCommand(
            trajectory,
            self.swerveSubsystem.getPose(),
            SwerveConstants.kDriveKinematics,
            x_pid,
            y_pid,
            theta_pid,
            self.swerveSubsystem.setModuleStates,
            [self.swerveSubsystem],
        )
        return SequentialCommandGroup(
            swerve_command,
            InstantCommand(self.swerveSubsystem.stop),
        )
        """


    def end(self, interrupted: bool) -> None:
        print("VisionTrack Interupted")
        # Switch back to april tag detection
        self.limelightTable.getEntry("pipeline").setInteger(LimelightConstants.apriltagPipelineID)


    def isFinished(self) -> bool:
        if self.targetPoseRobotSpaceArray == [0,0,0]:
            print("Äll Pose Values were 0")
            return True
        return False
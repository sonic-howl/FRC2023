import wpilib as wp
from commands2 import CommandScheduler

from constants.RobotConstants import RobotConstants
from RobotContainer import RobotContainer
from utils.utils import printAsync


class Robot(wp.TimedRobot):
    def __init__(self, period=RobotConstants.period) -> None:
        super().__init__(period)

    def robotInit(self) -> None:
        # wp.CameraServer.launch() # for camera plugged into RoboRIO

        RobotConstants.isSimulation = self.isSimulation()

        self.robotContainer = RobotContainer()

        self.swerveAutoCommand = self.robotContainer.getAutonomousCommand()

    def robotPeriodic(self) -> None:
        # The CommandScheduler automatically runs the appropriate code depending on the state of the competition (disabled, autonomous, teleoperated).
        try:
            CommandScheduler.getInstance().run()
        except Exception as e:
            printAsync("Caught exception:", e)

    def autonomousInit(self) -> None:
        self.swerveAutoCommand = self.robotContainer.getAutonomousCommand()
        self.swerveAutoCommand.schedule()

    def autonomousExit(self) -> None:
        self.swerveAutoCommand.cancel()

    def autonomousPeriodic(self) -> None:
        pass

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        print("Test mode initialized")

    def testPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wp.run(Robot)

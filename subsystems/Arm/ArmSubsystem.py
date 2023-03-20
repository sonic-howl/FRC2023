import ctre
import rev

from commands2 import SubsystemBase, ProfiledPIDSubsystem

from utils.conversions import Conversions
from constants import ArmConstants


class ArmSubsystem(SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        self.arm_motor = ctre.WPI_TalonSRX(10)
        self.arm_motor.setInverted(True)
        self.arm_motor.config_kP(0, ArmConstants.kArmP)
        self.arm_motor.config_kI(0, ArmConstants.kArmI)
        self.arm_motor.config_kD(0, ArmConstants.kArmD)
        self.arm_motor.config_kF(0, ArmConstants.kArmFF)

        self.claw_motor = rev.CANSparkMax(11, rev.CANSparkMax.MotorType.kBrushless)
        self.claw_motor.setInverted(True)
        self.claw_pid = self.claw_motor.getPIDController()
        # self.claw_pid.setP(ArmConstants.kClawP)
        # self.claw_pid.setI(ArmConstants.kClawI)
        # self.claw_pid.setD(ArmConstants.kClawD)
        # self.claw_pid.setIZone(ArmConstants.kClawIz)
        # self.claw_pid.setFF(ArmConstants.kClawFF)
        # self.claw_pid.setOutputRange(-1, 1)
        self.claw_encoder = self.claw_motor.getEncoder()
        # TODO
        # self.claw_encoder.setPositionConversionFactor()

    def getArmAngle(self):
        # return self.arm_motor.getSelectedSensorPosition() * 360 / 4096
        # return self.arm_motor.getSelectedSensorPosition() * 360 / 4096
        return Conversions.falconToDegrees(
            self.arm_motor.getSelectedSensorPosition(), ArmConstants.kArmReduction
        )

    def getClawAngle(self):
        """Returns the angle of the claw added to the angle of the arm to get the total angle of the claw"""
        return self.claw_encoder.getPosition() + self.getArmAngle()

    def setArmAngle(self, angle: float):
        # self.arm_motor.set(ctre.ControlMode.Position, angle * 4096 / 360)
        # self.arm_motor.set(
        #     ctre.ControlMode.Position,
        #     Conversions.degreesToFalcon(angle, ArmConstants.kArmReduction),
        #     ctre.DemandType.ArbitraryFeedForward,
        #     # ArmConstants.kArmFF,
        #     0,  # TODO get feedforward from wpilib ArmFeedForward class
        # )
        self.arm_motor.set(
            ctre.ControlMode.MotionMagic,
            Conversions.degreesToFalcon(angle, ArmConstants.kArmReduction),
            ctre.DemandType.ArbitraryFeedForward,
            # ArmConstants.kArmFF,
            0,  # TODO get feedforward from wpilib ArmFeedForward class
        )

    def setClawAngle(self, angle: float):
        # self.claw_pid.setReference(
        #     angle,
        #     rev.CANSparkMax.ControlType.kPosition,
        #     0,
        #     ArmConstants.kClawFF,  # TODO get feedforward from wpilib ArmFeedForward class
        # )
        self.claw_pid.setReference(
            angle,
            rev.CANSparkMax.ControlType.kSmartMotion,
            0,
            ArmConstants.kClawFF,  # TODO get feedforward from wpilib ArmFeedForward class
        )

    def setClawXY(self, x: float, y: float):
        # TODO this is probably not needed
        pass

    def stow(self):
        self.setArmAngle(0)
        self.setClawAngle(0)

from typing import Type
import math

import rev
from commands2 import SubsystemBase
from wpimath.controller import ArmFeedforward

from constants import ArmConstants, Constants


class ArmSubsystem(SubsystemBase):
    def __init__(self, constants: Type[ArmConstants.Arm | ArmConstants.Claw]) -> None:
        super().__init__()

        # arm
        self.armMotor = rev.CANSparkMax(
            constants.kCANId, rev.CANSparkMax.MotorType.kBrushed
        )
        self.armMotor.restoreFactoryDefaults()
        self.armMotor.setOpenLoopRampRate(0.5)
        self.armMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        # PID setup (not used)
        self.armPID = self.armMotor.getPIDController()
        self.armPID.setOutputRange(-Constants.max_speed, Constants.max_speed)
        self.armPID.setP(constants.kP)
        self.armPID.setI(constants.kI)
        self.armPID.setD(constants.kD)
        self.armPID.setIZone(constants.kIz)
        self.armPID.setFF(constants.kFF)
        # Smart Motion setup
        self.armPID.setSmartMotionMinOutputVelocity(0)
        self.armPID.setSmartMotionMaxVelocity(constants.kMaxVelocityRPM)
        self.armPID.setSmartMotionMaxAccel(constants.kMaxAccelerationRPM)
        self.armPID.setSmartMotionAccelStrategy(
            rev.SparkMaxPIDController.AccelStrategy.kSCurve
        )
        self.armPID.setSmartMotionAllowedClosedLoopError(0)
        # Arm encoder setup
        self.armEncoder = self.armMotor.getEncoder(
            rev.SparkMaxRelativeEncoder.Type.kQuadrature, 1024
        )
        self.armEncoder.setPositionConversionFactor(constants.kConversionFactor)
        self.armEncoder.setVelocityConversionFactor(constants.kConversionFactor / 60)
        self.armMotor.burnFlash()

        self.armFF = ArmFeedforward(
            constants.kS,
            constants.kG,
            constants.kV,
            constants.kA,
        )

    def setAngle(self, angle: float):
        """
        Sets the angle of the arm in degrees.
        :param angle: The angle to set the arm to in degrees.
        :param ffVoltage: The feedforward voltage to apply to the arm.
        """

        # TODO: maybe calculate acceleration too?
        velocity = self.armEncoder.getVelocity()
        ffVoltage = self.armFF.calculate(math.radians(angle), velocity)

        self.armPID.setReference(
            angle, rev.CANSparkMax.ControlType.kSmartMotion, arbFeedforward=ffVoltage
        )

    def getAngle(self):
        """
        Gets the angle of the arm in degrees.
        :return: The angle of the arm in degrees.
        """
        return self.armEncoder.getPosition() % 360

from typing import Type
import math

import wpilib

import rev
from commands2 import SubsystemBase
from wpimath.controller import ArmFeedforward

from constants import ArmConstants, Constants


class ArmSubsystem(SubsystemBase):
    def __init__(self, constants: Type[ArmConstants.Arm | ArmConstants.Claw]) -> None:
        super().__init__()

        self.initialPosition = constants.initialPosition

        # arm
        self.armMotor = rev.CANSparkMax(constants.kCANId, constants.motorType)
        self.armMotor.restoreFactoryDefaults()
        self.armMotor.setOpenLoopRampRate(0.5)
        self.armMotor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)

        # soft limits
        # ! test this. It may be raw encoder units rather than degrees (scaled from encoder units)
        # self.armMotor.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, True)
        # self.armMotor.setSoftLimit(
        #     rev.CANSparkMax.SoftLimitDirection.kForward, constants.kForwardSoftLimit
        # )
        # self.armMotor.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, True)
        # self.armMotor.setSoftLimit(
        #     rev.CANSparkMax.SoftLimitDirection.kReverse, constants.kReverseSoftLimit
        # )
        # self.armMotor.getForwardLimitSwitch(rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        # TODO handle logic in a periodic method to reset the encoder position if the limit switch is triggered

        # PID setup (not used)
        # ? or maybe it's used by Smart Motion?
        self.armPID = self.armMotor.getPIDController()
        self.armPID.setOutputRange(-Constants.maxSpeed, Constants.maxSpeed)
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
            *constants.getEncoderArgs
        )  # when brushed with data port encoder wire: rev.SparkMaxRelativeEncoder.Type.kQuadrature, 1024
        self.armEncoder.setPositionConversionFactor(constants.kConversionFactor)
        self.armEncoder.setVelocityConversionFactor(
            constants.kConversionFactor
            * (math.tau / 60)  # converting from RPM to radians per second
        )
        self.armEncoder.setPosition(self.initialPosition)

        self.armMotor.burnFlash()

        # feedforward setup
        self.armFF = ArmFeedforward(
            constants.kS,
            constants.kG,
            constants.kV,
            constants.kA,
        )

        # acceleration calculation
        self.timer = wpilib.Timer()
        self.timer.start()
        self.lastTime = float(self.timer.get())
        self.lastVelocity = 0.0
        self.acceleration = 0.0

    def periodic(self) -> None:
        # calculating acceleration periodically so it's always up to date for the feedforward controller
        # ? maybe this could be moved into setAngle()?
        velocity = self.armEncoder.getVelocity()
        now = float(self.timer.get())
        self.acceleration = (velocity - self.lastVelocity) / (now - self.lastTime)
        self.lastVelocity = velocity
        self.lastTime = now

        # set position to zero when there is speed being applied and the velocity is zero
        # if self.armMotor.get()
        # TODO test this
        # speedThreshold = 0.25
        # velocityThresholdRad = 0.01
        # if (
        #     abs(self.armMotor.getAppliedOutput()) > speedThreshold
        #     and abs(self.armEncoder.getVelocity()) < velocityThresholdRad
        # ):
        #     self.armEncoder.setPosition(self.initialPosition)

    def setAngle(self, angle: float):
        """
        Sets the angle of the arm in degrees.
        :param angle: The angle to set the arm to in degrees.
        :param ffVoltage: The feedforward voltage to apply to the arm.
        """
        velocity = self.armEncoder.getVelocity()
        ffVoltage = self.armFF.calculate(
            math.radians(angle), velocity, self.acceleration
        )

        self.armPID.setReference(
            angle, rev.CANSparkMax.ControlType.kSmartMotion, arbFeedforward=ffVoltage
        )

    def getAngle(self):
        """
        Gets the angle of the arm in degrees.
        :return: The angle of the arm in degrees.
        """
        return self.armEncoder.getPosition() % 360

    def addAngle(self, angle: float):
        """
        Adds an angle to the current angle of the arm in degrees.
        :param angle: The angle to add to the arm in degrees.
        """
        self.setAngle(self.getAngle() + angle)

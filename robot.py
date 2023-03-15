import wpilib as wp
import rev
import math
import ctre
from wpimath.controller import PIDController

class Robot(wp.TimedRobot):
    def robotInit(self):
        # self.motor = rev.CANSparkMax(2, rev.CANSparkMax.MotorType.kBrushless)
        # self.armPID = PIDController(0.02, 0, 0)
        # self.motorEncoder = self.motor.getAbsoluteEncoder(rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        # self.armAngleSetPoint = self.motorEncoder.getPosition()
        # self.turn = rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless)
        # self.turnEnconder = self.turn.getAbsoluteEncoder(rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.controller = wp.XboxController(0)
        # kP = 0.05
        # self.turnPID = PIDController(kP, 0, 0)
        self.armmotor = ctre.TalonSRX(10)
    
    def teleopPeriodic(self):
        # L2 = self.controller.getLeftTriggerAxis()
        # R2 = self.controller.getRightTriggerAxis()
        # armAngle = self.motorEncoder.getPosition() # assuming it's between 0 and 2pi
        # armAngle -= math.pi
        # if abs(armAngle) < 0.87:
        #     if L2 > .5:
        #         self.armAngleSetPoint -= .0000001
        #     if R2 > .5:
        #         self.armAngleSetPoint += .0000001
        # armMotorSpeed = self.armPID.calculate(armAngle, self.armAngleSetPoint)
        # self.motor.set(armMotorSpeed)
        # y = self.controller.getRightY()
        # x = self.controller.getRightX()
        # angle = math.atan2(y, x) + math.pi
        # angle += math.pi 
        # angle = angle % (math.pi * 2)
        # currentAngle = self.turnEnconder.getPosition()
        # print("angle", angle, "currentAngle", currentAngle)
        # motorturnSpeed = self.turnPID.calculate(currentAngle, angle)
        # self.turn.set(motorturnSpeed)
        armspeed = -self.controller.getLeftY()
        self.armmotor.set(ctre.ControlMode.PercentOutput,armspeed)



    def robotPeriodic(self):
        pass
    
if __name__ == '__main__':
    wp.run(Robot)
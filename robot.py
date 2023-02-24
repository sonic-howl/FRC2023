import wpilib as wp
import rev
import math
from wpimath.controller import PIDController

class Robot(wp.TimedRobot):
    def robotInit(self):
        self.motor = rev.CANSparkMax(2, rev.CANSparkMax.MotorType.kBrushless)
        self.turn = rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless)
        self.turnEnconder = self.turn.getAbsoluteEncoder(rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.controller = wp.XboxController(0)
        kP = 0.05
        self.turnPID = PIDController(kP, 0, 0)
    
    def teleopPeriodic(self):
       
        y = self.controller.getRightY()
        x = self.controller.getRightX()
        angle = math.atan2(y, x) + math.pi
        angle += math.pi 
        angle = angle % (math.pi * 2)
        currentAngle = self.turnEnconder.getPosition()
        print("angle", angle, "currentAngle", currentAngle)
        motorturnSpeed = self.turnPID.calculate(currentAngle, angle)
        self.turn.set(motorturnSpeed)

        # print(angle)
        # motorSpeed = self.controller.getRawAxis(1)
        # motorTurnSpeed = self.controller.getRawAxis(0)
        # if motorSpeed < 0.05:
        #     motorSpeed = 0
        # if motorTurnSpeed < 0.05:
        #     motorTurnSpeed = 0
        # self.motor.set(motorSpeed)
        # self.turn.set(motorTurnSpeed)

    def robotPeriodic(self):
        pass
    
if __name__ == '__main__':
    wp.run(Robot)
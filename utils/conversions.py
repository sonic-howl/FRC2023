import math


class UnitConversions:
    @staticmethod
    def rpmToRps(rpm: float):
        """
        Convert RPM to radians per second.
        """
        rps = rpm * math.tau / 60
        return rps


class MotorConversions:
    @staticmethod
    def CANcoderToDegrees(positionCounts: float, gearRatio: float):
        """
        :param positionCounts: CANCoder Position Counts
        :param gearRatio: Gear Ratio between CANCoder and Mechanism
        :return: Degrees of Rotation of Mechanism
        """
        return positionCounts * (360.0 / (gearRatio * 4096.0))

    @staticmethod
    def degreesToCANcoder(degrees: float, gearRatio: float):
        """
        :param degrees: Degrees of rotation of Mechanism
        :param gearRatio: Gear Ratio between CANCoder and Mechanism
        :return: CANCoder Position Counts
        """
        return degrees / (360.0 / (gearRatio * 4096.0))

    @staticmethod
    def falconToDegrees(positionCounts: float, gearRatio: float):
        """
        :param positionCounts: Falcon Position Counts
        :param gearRatio: Gear Ratio between Falcon and Mechanism
        :return: Degrees of Rotation of Mechanism
        """
        return positionCounts * (360.0 / (gearRatio * 2048.0))

    @staticmethod
    def degreesToFalcon(degrees: float, gearRatio: float):
        """
        :param degrees: Degrees of rotation of Mechanism
        :param gearRatio: Gear Ratio between Falcon and Mechanism
        :return: Falcon Position Counts
        """
        return degrees / (360.0 / (gearRatio * 2048.0))

    @staticmethod
    def falconToRPM(velocityCounts: float, gearRatio: float):
        """
        :param velocityCounts: Falcon Velocity Counts
        :param gearRatio: Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
        :return: RPM of Mechanism
        """
        motorRPM = velocityCounts * (600.0 / 2048.0)
        mechRPM = motorRPM / gearRatio
        return mechRPM

    @staticmethod
    def RPMToFalcon(RPM: float, gearRatio: float):
        """
        :param RPM: RPM of mechanism
        :param gearRatio: Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
        :return: Falcon Velocity Counts
        """
        motorRPM = RPM * gearRatio
        sensorCounts = motorRPM * (2048.0 / 600.0)
        return sensorCounts

    @staticmethod
    def falconToMPS(velocityCounts: float, circumference: float, gearRatio: float):
        """
        :param velocityCounts: Falcon Velocity Counts
        :param circumference: Circumference of Wheel
        :param gearRatio: Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
        :return: Falcon Velocity Counts
        """
        wheelRPM = MotorConversions.falconToRPM(velocityCounts, gearRatio)
        wheelMPS = (wheelRPM * circumference) / 60
        return wheelMPS

    @staticmethod
    def MPSToFalcon(velocity: float, circumference: float, gearRatio: float):
        """
        :param velocity: Velocity MPS
        :param circumference: Circumference of Wheel
        :param gearRatio: Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
        :return: Falcon Velocity Counts
        """
        wheelRPM = (velocity * 60) / circumference
        wheelVelocity = MotorConversions.RPMToFalcon(wheelRPM, gearRatio)
        return wheelVelocity

    @staticmethod
    def falconToMeters(positionCounts: float, circumference: float, gearRatio: float):
        """
        convert falcon encoder units per 100ms to meters
        :param positionCounts: Falcon Position Counts
        :param circumference: Circumference of Wheel
        :param gearRatio: Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
        :return: Meters
        """
        return positionCounts * (circumference / (gearRatio * 2048.0))

    @staticmethod
    def metersToFalcon(meters: float, circumference: float, gearRatio: float) -> float:
        """
        convert meters to falcon encoder units per 100ms
        :param meters: Meters
        :param circumference: Circumference of Wheel
        :param gearRatio: gearRatio Gear Ratio between Falcon and Wheel
        :return: Falcon Position Counts
        """
        return meters / (circumference / (gearRatio * 2048.0))

    @staticmethod
    def versaPlanetaryToDegrees(positionCounts: float, gearRatio: float):
        """
        :param positionCounts: VersaPlanetary Position Counts
        :param gearRatio: Gear Ratio between VersaPlanetary and Mechanism
        :return: Degrees of Rotation of Mechanism
        """
        return positionCounts * (360.0 / (gearRatio * 1024.0))

    @staticmethod
    def degreesToVersaPlanetary(degrees: float, gearRatio: float):
        """
        :param degrees: Degrees of rotation of Mechanism
        :param gearRatio: Gear Ratio between VersaPlanetary and Mechanism
        :return: VersaPlanetary Position Counts
        """
        return degrees / (360.0 / (gearRatio * 1024.0))

    @staticmethod
    def versaPlanetaryToRPM(velocityCounts: float, gearRatio: float):
        """
        :param velocityCounts: VersaPlanetary Velocity Counts
        :param gearRatio: Gear Ratio between VersaPlanetary and Mechanism (set to 1 for VersaPlanetary RPM)
        :return: RPM of Mechanism
        """
        motorRPM = velocityCounts * (600.0 / 1024.0)
        mechRPM = motorRPM / gearRatio
        return mechRPM

    @staticmethod
    def RPMToVersaPlanetary(RPM: float, gearRatio: float):
        """
        :param RPM: RPM of mechanism
        :param gearRatio: Gear Ratio between VersaPlanetary and Mechanism (set to 1 for VersaPlanetary RPM)
        :return: VersaPlanetary Velocity Counts
        """
        motorRPM = RPM * gearRatio
        sensorCounts = motorRPM * (1024.0 / 600.0)
        return sensorCounts

    @staticmethod
    def versaPlanetaryToMPS(
        velocityCounts: float, circumference: float, gearRatio: float
    ):
        """
        :param velocityCounts: VersaPlanetary Velocity Counts
        :param circumference: Circumference of Wheel
        :param gearRatio: Gear Ratio between VersaPlanetary and Mechanism (set to 1 for VersaPlanetary MPS)
        :return: VersaPlanetary Velocity Counts
        """
        wheelRPM = MotorConversions.versaPlanetaryToRPM(velocityCounts, gearRatio)
        wheelMPS = (wheelRPM * circumference) / 60
        return wheelMPS

    @staticmethod
    def MPSToVersaPlanetary(velocity: float, circumference: float, gearRatio: float):
        """
        :param velocity: Velocity MPS
        :param circumference: Circumference of Wheel
        :param gearRatio: Gear Ratio between VersaPlanetary and Mechanism (set to 1 for VersaPlanetary MPS)
        :return: VersaPlanetary Velocity Counts
        """
        wheelRPM = (velocity * 60) / circumference
        wheelVelocity = MotorConversions.RPMToVersaPlanetary(wheelRPM, gearRatio)
        return wheelVelocity

    @staticmethod
    def versaPlanetaryToMeters(
        positionCounts: float, circumference: float, gearRatio: float
    ):
        """
        convert VersaPlanetary encoder units per 100ms to meters
        :param positionCounts: VersaPlanetary Position Counts
        :param circumference: Circumference of Wheel
        :param gearRatio: Gear Ratio between VersaPlanetary and Mechanism (set to 1 for VersaPlanetary MPS)
        :return: Meters
        """
        return positionCounts * (circumference / (gearRatio * 1024.0))

    @staticmethod
    def metersToVersaPlanetary(
        meters: float, circumference: float, gearRatio: float
    ) -> float:
        """
        convert meters to VersaPlanetary encoder units per 100ms
        :param meters: Meters
        :param circumference: Circumference of Wheel
        :param gearRatio: gearRatio Gear Ratio between VersaPlanetary and Wheel
        :return: VersaPlanetary Position Counts
        """
        return meters / (circumference / (gearRatio * 1024.0))

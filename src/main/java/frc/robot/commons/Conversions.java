package frc.robot.commons;

public class Conversions {

    public static double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));
    }

    public static double degreesToFalcon(double degrees, double gearRatio) {
        double ticks =  degrees / (360.0 / (gearRatio * 2048.0));
        return ticks;
    }

    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    public static double MPSToFalcon(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    public static double CANCoderSensorUnitsToDegrees(double sensorUnits, double gearRatio) {
        return sensorUnits * (360.0) / 4096.0;
    }

    public static double degreesToCANCoderSensorUnits(double degrees, double gearRatio) {
        return degrees * 4096.0 / (360.0);
    }

    public static double CANCoderSensorUnitsToDegreesPerSecond(double sensorUnits, double gearRatio) {
        return sensorUnits * ((360.0 * 10.0)/4096.0);
    }

    public static double degreesPerSecondToCANCoderSensorUnits(double degrees, double gearRatio) {
        return degrees * (4096.0/(360.0 * 10.0));
    }

    public static double percentOutputPerDegreeToCANCoderKP(double percentOutputPerDegree, double gearRatio) {
        return percentOutputPerDegree * 360.0/4096.0 * 1023.0;
    }

    // public static double percentOutputPerDegreePerSecondToCANCoderKD(double percentOutputPerDegreePerSecond, double gearRatio) {
    //     return percentOutputPerDegreePerSecond *
    // }

}
package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTempCelcius = 0.0;
        public double driveDistanceMeters = 0.0;
        public double driveOutputPercent = 0.0;
        public double rawDriveRPM = 0.0;

        public double moduleAngleRads = 0.0;
        public double rawAbsolutePositionDegrees = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0; 
        public double turnTempCelcius = 0.0;
    }

    /** Updates the set of loggable inputs */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at a specified velocity */
    public default void setDriveVelocity(double velocityMetersPerSecond, boolean auto) {}

    /** Run the drive motor at a specified percent */
    public default void setDrivePercent(double percent) {}

    /** Set the turn motor to a particular angle */
    public default void setTurnAngle(double positionRads) {}

    /** Enable or disable drive brake mode */
    public default void setDriveBrakeMode(boolean enable) {}

    /** Enable or disable turn brake mode */
    public default void setTurnBrakeMode(boolean enable) {}

    /** Resets the turn motor to its absolute position */
    public default void resetToAbsolute() {}

    /** Updates set of tunable numbers */
    public default void updateTunableNumbers() {}
}

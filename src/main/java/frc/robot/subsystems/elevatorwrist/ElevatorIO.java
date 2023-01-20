package frc.robot.subsystems.elevatorwrist;

import org.littletonrobotics.junction.AutoLog;

/* Elevator subsystem hardware interface */
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double posMeters = 0.0;
        public double velMetersPerSecond = 0.0;
        public double appliedVoltage = 0.0;
        public double[] currentAmps = new double[] {}; // {leader, follower}
        public double[] tempCelcius = new double[] {}; // {leader, follower}
    }

    /* Updates the set of loggable inputs */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /* Sets the climber to a height setpoint via motion magic */
    public default void setHeight(double heightMeters) {}

    /** Sets the climber to a specified percent output */
    public default void setPercent(double percent) {}

    /* Sets the climber's neutral mode */
    public default void enableBrakeMode(boolean enable) {}

}
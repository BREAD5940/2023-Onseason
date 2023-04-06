package frc.robot.subsystems.elevatorarm;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.ErrorCode;


/* Elevator subsystem hardware interface */
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double posMeters = 0.0;
        public double velMetersPerSecond = 0.0;
        public double velTarget = 0.0;
        public double posTarget = 0.0;
        public double appliedVoltage = 0.0;
        public double[] currentAmps = new double[] {}; // {leader, follower}
        public double[] tempCelcius = new double[] {}; // {leader, follower}

        public String lastLeaderError = ErrorCode.OK.toString();
        public String lastFollowerError = ErrorCode.OK.toString();
    }

    /* Updates the set of loggable inputs */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /* Sets the climber to a height setpoint via motion magic */
    public default void setHeight(double heightMeters, boolean goSlow) {}

    /** Sets the climber to a specified percent output */
    public default void setPercent(double percent) {}

    /** Resets the climber to a specified position */
    public default void resetHeight(double newHeightMeters) {}

    /* Sets the climber's neutral mode */
    public default void enableBrakeMode(boolean enable) {}

    /* Updates tunable numbers if neccesary */
    public default void updateTunableNumbers() {}

}
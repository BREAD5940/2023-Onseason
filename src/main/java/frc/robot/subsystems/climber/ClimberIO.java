package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/* Climber subsystem hardware interface */
public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double heightMeters = 0.0;
        public double velMetersPerSecond = 0.0;
        public double appliedVoltage = 0.0;
        public boolean forksDeployed = false;
        public double[] currentAmps = new double[] {}; // {leader, follower}
        public double[] tempCelcius = new double[] {}; // {leader, follower}
    }

    /* Updates the set of loggable inputs */
    public default void updateInputs(ClimberIOInputs inputs) { }

    /* Sets the percent of the climber */
    public default void setPercent(double percent) { }

    /* Sets the climber to a certain height */
    public default void setHeight(double height) { }

    /* Enables brake mode on the climber motors */
    public default void enableBrakeMode(boolean enable) { }    
    
}

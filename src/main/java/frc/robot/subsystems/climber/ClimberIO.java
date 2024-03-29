package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/* Climber subsystem hardware interface */
public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double heightMeters = 0.0;
        public double appliedVoltage = 0.0;
        public boolean forksDeployed = false;
        public double currentAmps = 0.0;
        public double tempCelcius = 0.0;
    }

    /* Updates the set of loggable inputs */
    public default void updateInputs(ClimberIOInputs inputs) { }

    /* Sets the percent of the climber */
    public default void setPercent(double percent) { }

    /* Sets the climber to a certain height */
    public default void setHeight(double height) { }

    /* Sets the currents for the climber */
    public default void setCurrentLimits(double currentLimit, double currentLimitTriggerThreshold, double currentLimitThresholdTime) { }

    /* Enables brake mode on the climber motors */
    public default void enableBrakeMode(boolean enable) { }    
    
}

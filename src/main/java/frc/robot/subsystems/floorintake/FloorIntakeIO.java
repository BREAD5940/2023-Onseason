package frc.robot.subsystems.floorintake;

import org.littletonrobotics.junction.AutoLog;

public interface FloorIntakeIO {
    @AutoLog
    public static class FloorIntakeIOInputs {
        public double rollerCurrentAmps = 0.0;
        public double rollerAppliedVoltage = 0.0;
        public double rollerTempCelcius = 0.0;

        public double deployVelDegreesPerSecond = 0.0;
        public double deployAngleDegrees = 0.0;
        public double deployCurrentAmps = 0.0;
        public double deployAppliedVoltage = 0.0;
        public double deployTempCelcius = 0.0;
    }

    /** Updates the set of loggable inputs */
    public default void updateInputs(FloorIntakeIOInputs inputs) {}

    /** Sets the desired percent output on the intake roller */
    public default void setRollerPercent(double percent) {}
        
    /** Sets desired deploy angle */  
    public default void setDeployAngle(double angle) {}

    /** Sets percent for deploy motor  */
    public default void setDeployPercent(double percent) {}

    /** Sets current limit for deploy motor */
    public default void setCurrentLimit(double currentLimit, double currentLimitTriggerThreshold, double currentLimitThresholdTime) {}

    /** Enables or disables brake mode for the roller */
    public default void enableRollerBrakeMode(boolean enable) {}

    /** Enables or disables brake mode for the deploy motor */
    public default void enableDeployBrakeMode(boolean enable) {}

}
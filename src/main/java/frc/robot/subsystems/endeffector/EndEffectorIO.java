package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorIOInputs {
        public double currentAmps = 0.0;
        public double appliedVoltage = 0.0;
    }

    /** Updates the set of loggable inputs */
    public default void updateInputs(EndEffectorIOInputs inputs) {}

    /** Sets the desired percent output on the end effector */
    public default void setPercent(double percent) {}

    /** Enables or disables brake mode for the end effector */
    public default void enableBrakeMode(boolean enable) {}
    
}
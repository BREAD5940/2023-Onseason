package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.ErrorCode;

public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorIOInputs {
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double avgStatorCurrentAmps = 0.0;
        public double appliedVoltage = 0.0;
        public double tempCelcius = 0.0;
        public double motorSpeed = 0.0;

        public String lastError = ErrorCode.OK.toString();
    }

    /** Updates the set of loggable inputs */
    public default void updateInputs(EndEffectorIOInputs inputs) {}

    /** Sets the desired percent output on the end effector */
    public default void setPercent(double percent) {}

    /** Enables or disables brake mode for the end effector */
    public default void enableBrakeMode(boolean enable) {}

    /** Sets current limit of end-effector */
    public default void setCurrentLimit(double currentLimit, double triggerThreshhold) {}

    /** Updates filters */
    public default void updateFilter() {}

    /** resets sticky faults to allow error to change from anything back to "ok" */
    public default void clearFault(){}
    
	/**
	 * used to check if the end effector is holding a cone
	 * @return
	 * returns true if the end effector is holding a cone
	 */
	public default boolean isHoldingCone() {return false;}
}
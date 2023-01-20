package frc.robot.subsystems.floorintake;

import org.littletonrobotics.junction.AutoLog;

public interface FloorIntakeIO {
    @AutoLog
    public static class FloorIntakeIOInputs {
        public double velRPM = 0.0;
        public double currentAmps = 0.0;
        public double appliedVoltage = 0.0;
        public double tempCelcius = 0.0;
    }

      /** Updates the set of loggable inputs */
      public default void updateInputs(FloorIntakeIOInputs inputs) {}

      /** Sets the desired percent output on the end effector */
      public default void setPercent(double percent) {}
  
      /** Enables or disables brake mode for the end effector */
      public default void enableBrakeMode(boolean enable) {}
}
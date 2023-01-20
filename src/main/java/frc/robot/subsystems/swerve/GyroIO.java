package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double positionDegRaw = 0.0;
        public double velocityRadPerSec = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void reset() {}    
}
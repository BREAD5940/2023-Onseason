package frc.robot.subsystems.endeffector;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;

import static frc.robot.Constants.EndEffector.*;

public class EndEffectorIOSparkMax implements EndEffectorIO {
    CANSparkMax motor;

    public EndEffectorIOSparkMax() {
        motor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
        motor.setInverted(IS_REVERSED);
        motor.setSmartCurrentLimit(7);
        motor.setSecondaryCurrentLimit(10);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.appliedVoltage = motor.getAppliedOutput() * RobotController.getBatteryVoltage(); 
        inputs.currentAmps = motor.getOutputCurrent();
    }

    @Override
    public void setPercent(double percent) {
        motor.set(percent);
    }

    @Override
    public void enableBrakeMode(boolean enable) {
        motor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
}


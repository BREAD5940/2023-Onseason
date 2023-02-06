package frc.robot.subsystems.endeffector;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.commons.AveragingFilter;

import static frc.robot.Constants.EndEffector.*;

public class EndEffectorIOSparkMax implements EndEffectorIO {

    CANSparkMax motor;
    AveragingFilter filter = new AveragingFilter(13);

    public EndEffectorIOSparkMax() {
        motor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
        motor.setInverted(IS_REVERSED);
        motor.setSmartCurrentLimit(5);
        motor.setSecondaryCurrentLimit(7);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.appliedVoltage = motor.getAppliedOutput() * RobotController.getBatteryVoltage(); 
        inputs.statorCurrentAmps = motor.getOutputCurrent();
        inputs.avgStatorCurrentAmps = filter.getAverage();
        inputs.tempCelcius = motor.getMotorTemperature();
    }

    @Override
    public void setPercent(double percent) {
        motor.set(percent);
    }

    @Override
    public void enableBrakeMode(boolean enable) {
        motor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setCurrentLimit(int smartCurrent, double secondaryCurrent) {
        motor.setSmartCurrentLimit(smartCurrent);
        motor.setSecondaryCurrentLimit(secondaryCurrent);
    }

    @Override
    public void updateFilter() {
        filter.addSample(motor.getOutputCurrent());
    }
}


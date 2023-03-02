package frc.robot.subsystems.endeffector;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.commons.AveragingFilter;

import static frc.robot.Constants.EndEffector.*;
import static frc.robot.Constants.Electrical.*;

public class EndEffectorIOTalonFX implements EndEffectorIO {
    private TalonFX motor;
    private AveragingFilter filter = new AveragingFilter(13);
    private double mCurrentLimit = 0.0;


    public EndEffectorIOTalonFX() {
        motor = new TalonFX(MOTOR_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.supplyCurrLimit.enable = true;
        config.supplyCurrLimit.triggerThresholdCurrent = 40;
        config.supplyCurrLimit.currentLimit = 40;
        config.supplyCurrLimit.triggerThresholdTime = 1.5;
        config.voltageCompSaturation = 9.0;
        motor.enableVoltageCompensation(true);
        motor.configAllSettings(config);
        motor.setInverted(INVERSION);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.appliedVoltage = motor.getMotorOutputPercent() * RobotController.getBatteryVoltage(); 
        inputs.statorCurrentAmps = Math.abs(motor.getStatorCurrent());
        inputs.supplyCurrentAmps = motor.getSupplyCurrent();
        inputs.avgStatorCurrentAmps = filter.getAverage();
        inputs.tempCelcius = motor.getTemperature();
    }

    @Override
    public void setPercent(double percent) {
        motor.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void enableBrakeMode(boolean enable) {
        motor.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public void setCurrentLimit(double currentLimit, double triggerThreshhold) {
        if (currentLimit != mCurrentLimit) {
            motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit, triggerThreshhold, 1.5));
            mCurrentLimit = currentLimit;
        }
    }

    @Override
    public void updateFilter() {
        filter.addSample(Math.abs(motor.getStatorCurrent()));
    }
}


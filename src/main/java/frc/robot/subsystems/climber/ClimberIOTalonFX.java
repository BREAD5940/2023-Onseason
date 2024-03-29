package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.commons.LoggedTunableNumber;


import static frc.robot.Constants.Climber.*;

public class ClimberIOTalonFX implements ClimberIO {

    LoggedTunableNumber climbingFF = new LoggedTunableNumber("ClimbingFF", CLIMBING_FF);
    
    /** Instantiate the hardware */
    private TalonFX climber = new TalonFX(CLIMBER_ID, "dabus");

    /** Current limit variables */
    private double mCurrentLimit = 0.0;
    private double mCurrentLimitTriggerThreshhold = 0.0;
    private double mcurrentLimitThresholdTime = 0.0;

    public ClimberIOTalonFX() {
        /* Climber configs */
        TalonFXConfiguration climberConfig = new TalonFXConfiguration();
        climberConfig.slot0.kP = integratedSensorUnitsToMeters(0.03) * 1023.0;
        climberConfig.slot0.kI = integratedSensorUnitsToMeters(0) * 1023.0;
        climberConfig.slot0.kD = integratedSensorUnitsToMeters(0.003) * 1023.0;
        climberConfig.voltageCompSaturation = 10.0;
        climberConfig.neutralDeadband = 0.001;
        climberConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 60, 70, 1.5);
        climber.setInverted(CLIMBER_INVERT_TYPE);
        climber.setNeutralMode(NeutralMode.Coast);
        climber.setSensorPhase(true);
        climber.enableVoltageCompensation(true);
        climber.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        climber.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
        climber.set(ControlMode.PercentOutput, 0.0);
        climber.configAllSettings(climberConfig);
        climber.setSelectedSensorPosition(0.0);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.appliedVoltage = climber.getMotorOutputVoltage();
        inputs.currentAmps = climber.getStatorCurrent();
        inputs.forksDeployed = false;
        inputs.heightMeters = integratedSensorUnitsToMeters(climber.getSelectedSensorPosition());
        inputs.tempCelcius = climber.getTemperature();
    }

    @Override
    public void setPercent(double percent) {
        climber.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void setHeight(double height) {
        climber.set(ControlMode.Position, metersToIntegratedSensorUnits(height));
    }

    @Override
    public void setCurrentLimits(double currentLimit, double currentLimitTriggerThreshold, double currentLimitThresholdTime) {
        if (currentLimit != mCurrentLimit || currentLimitTriggerThreshold != mCurrentLimitTriggerThreshhold || currentLimitThresholdTime != mcurrentLimitThresholdTime) {
            mCurrentLimit = currentLimit;
            mCurrentLimitTriggerThreshhold = currentLimitTriggerThreshold;
            mcurrentLimitThresholdTime = currentLimitThresholdTime;
            climber.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit, currentLimitTriggerThreshold, currentLimitThresholdTime));
        }
    }

    @Override
    public void enableBrakeMode(boolean enable) {
        climber.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    private double integratedSensorUnitsToMeters(double integratedSensorUnits) {
        return integratedSensorUnits * ((CLIMBER_GEARING * Math.PI * CLIMBER_PULLEY_DIAMETER)/2048.0);
    }

    private double metersToIntegratedSensorUnits(double meters) {
        return meters * (2048.0/(CLIMBER_GEARING * Math.PI * CLIMBER_PULLEY_DIAMETER));
    }
    
}

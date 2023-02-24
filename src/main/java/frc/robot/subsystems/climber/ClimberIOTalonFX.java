package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.commons.TunableNumber;

import static frc.robot.Constants.Climber.*;

public class ClimberIOTalonFX implements ClimberIO {

    TunableNumber climbingFF = new TunableNumber("ClimbingFF", CLIMBING_FF);
    
    /** Instantiate the hardware */
    private TalonFX climber = new TalonFX(CLIMBER_ID);

    public ClimberIOTalonFX() {
        /* Climber configs */
        TalonFXConfiguration climberConfig = new TalonFXConfiguration();
        climberConfig.slot0.kP = integratedSensorUnitsToMeters(0.03) * 1023.0;
        climberConfig.slot0.kI = integratedSensorUnitsToMeters(0) * 1023.0;
        climberConfig.slot0.kD = integratedSensorUnitsToMeters(0.003) * 1023.0;
        climberConfig.voltageCompSaturation = 10.0;
        climberConfig.neutralDeadband = 0.001;
        climberConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 50, 50, 1.5);
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
    public void setPercent(double percent) {
        climber.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void setHeight(double height) {
        climber.set(ControlMode.Position, metersToIntegratedSensorUnits(height), DemandType.ArbitraryFeedForward, CLIMBING_FF);
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

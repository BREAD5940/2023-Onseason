package frc.robot.subsystems.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import static frc.robot.Constants.Elevator.*;

public class ElevatorIOTalonFX implements ElevatorIO {

    TalonFX leader = new TalonFX(ELEVATOR_LEFT_ID);
    TalonFX follower = new TalonFX(ELEVATOR_RIGHT_ID);

    public ElevatorIOTalonFX() {
        /* configurations for the leader motor */
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.slot0.kP = integratedSensorUnitsToMetersPerSecond(1.0) * 1023.0;
        leaderConfig.slot0.kI = integratedSensorUnitsToMetersPerSecond(0) * 1023.0;
        leaderConfig.slot0.kD = integratedSensorUnitsToMetersPerSecond(0.05) * 1023.0;
        leaderConfig.slot0.kF = 1023.0/metersPerSecondToIntegratedSensorUnits(1.301496);
        leaderConfig.motionCruiseVelocity = metersPerSecondToIntegratedSensorUnits(1.301496);
        leaderConfig.motionAcceleration = metersPerSecondToIntegratedSensorUnits(12.0);
        leaderConfig.voltageCompSaturation = 10.5;
        leaderConfig.neutralDeadband = 0.001;
        leaderConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 50, 50, 1.5);
        leader.setInverted(TalonFXInvertType.CounterClockwise);
        leader.setNeutralMode(NeutralMode.Brake);
        leader.enableVoltageCompensation(true);
        leader.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        leader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
        leader.configAllSettings(leaderConfig);
        leader.setSelectedSensorPosition(0.0);

        /* configurations for the follower motor */
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.neutralDeadband = 0.001;
        followerConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 50, 50, 1.5);
        follower.configAllSettings(followerConfig);
        follower.follow(leader);
        follower.setInverted(TalonFXInvertType.Clockwise);
        follower.setStatusFramePeriod(StatusFrame.Status_1_General, 197);
        follower.setNeutralMode(NeutralMode.Brake);
        follower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 193);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.posMeters = getHeight();
        inputs.velMetersPerSecond = integratedSensorUnitsToMetersPerSecond(leader.getSelectedSensorVelocity());
        inputs.appliedVoltage = leader.getMotorOutputVoltage();
        inputs.currentAmps = new double[] {leader.getStatorCurrent(), follower.getStatorCurrent()};
        inputs.tempCelcius = new double[] {leader.getTemperature(), follower.getTemperature()};
    }

    @Override
    public void setHeight(double heightMeters) {
        if (heightMeters > getHeight()) {
            leader.set(ControlMode.Position, metersToIntegratedSensorUnits(heightMeters), DemandType.ArbitraryFeedForward, GRAVITY_FF);
        } else {
            leader.set(ControlMode.Position, metersToIntegratedSensorUnits(heightMeters));
        }
    }

    @Override
    public void setPercent(double percent) {
        leader.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void resetHeight(double newHeightMeters) {
        leader.setSelectedSensorPosition(newHeightMeters);
    }

    @Override
    public void enableBrakeMode(boolean enable) {
        leader.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
        follower.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /* converts integrated sensor units to meters */
    private double integratedSensorUnitsToMeters(double integratedSensorUnits) {
        return integratedSensorUnits * ((0.13636363636 * Math.PI * 0.028575)/2048.0);
    }

    /* converts meters to integrated sensor units */
    private double metersToIntegratedSensorUnits(double meters) {
        return meters * (2048.0/(0.13636363636 * Math.PI * 0.028575));
    }   

    /* converts integrated sensor units to meters per second */
    private double integratedSensorUnitsToMetersPerSecond(double integratedSensorUnits) {
        return integratedSensorUnits * ((0.13636363636 * (600.0/2048.0) * Math.PI * 0.028575)/60.0);
    }

    /* converts meters per second to integrated sensor units */
    private double metersPerSecondToIntegratedSensorUnits(double metersPerSecond) {
        return metersPerSecond * (60.0/(0.13636363636 * (600.0/2048.0) * Math.PI * 0.028575));
    }

    /* returns the height of the climber in meters */
    private double getHeight() {
        return integratedSensorUnitsToMeters(leader.getSelectedSensorPosition());
    }
    
}

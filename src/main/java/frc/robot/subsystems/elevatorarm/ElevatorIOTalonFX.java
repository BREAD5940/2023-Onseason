package frc.robot.subsystems.elevatorarm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.commons.TunableNumber;

import static frc.robot.Constants.Elevator.*;
import static frc.robot.Constants.Electrical.*;

public class ElevatorIOTalonFX implements ElevatorIO {

    TalonFX leader = new TalonFX(ELEVATOR_LEFT_ID, CANIVORE_BUS_NAME);
    TalonFX follower = new TalonFX(ELEVATOR_RIGHT_ID, CANIVORE_BUS_NAME);

    TunableNumber kP = new TunableNumber("Elevator/kP", 0.4);
    TunableNumber kD = new TunableNumber("Elevator/kD", 0.075);
    TunableNumber kF = new TunableNumber("Elevator/kF", 2.7);
    TunableNumber kMaxVelocity = new TunableNumber("Elevator/kMaxVelocity", 2.1); 
    TunableNumber kMaxAccel = new TunableNumber("Elevator/kMaxAccel", 6.0); 


    public ElevatorIOTalonFX() {
        /* configurations for the leader motor */
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.slot0.kP = integratedSensorUnitsToMetersPerSecond(kP.get()) * 1023.0;
        leaderConfig.slot0.kI = integratedSensorUnitsToMetersPerSecond(0) * 1023.0;
        leaderConfig.slot0.kD = integratedSensorUnitsToMetersPerSecond(kD.get()) * 1023.0;
        leaderConfig.slot0.kF = 1023.0/metersPerSecondToIntegratedSensorUnits(kF.get());
        leaderConfig.motionCruiseVelocity = metersPerSecondToIntegratedSensorUnits(kMaxVelocity.get());
        leaderConfig.motionAcceleration = metersPerSecondToIntegratedSensorUnits(kMaxAccel.get());
        leaderConfig.voltageCompSaturation = 10.5;
        leaderConfig.neutralDeadband = 0.001;
        leaderConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 50, 50, 1.5);
        leader.setInverted(TalonFXInvertType.CounterClockwise);
        leader.setNeutralMode(NeutralMode.Brake);
        leader.enableVoltageCompensation(true);
        leader.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        leader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
        leader.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10);
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
        inputs.posMeters = getHeight();leader.getActiveTrajectoryVelocity();
        inputs.velMetersPerSecond = integratedSensorUnitsToMetersPerSecond(leader.getSelectedSensorVelocity());
        inputs.velTarget = integratedSensorUnitsToMetersPerSecond(leader.getActiveTrajectoryVelocity());
        inputs.posTarget = integratedSensorUnitsToMeters(leader.getActiveTrajectoryPosition());
        inputs.appliedVoltage = leader.getMotorOutputVoltage();
        inputs.currentAmps = new double[] {leader.getStatorCurrent(), follower.getStatorCurrent()};
        inputs.tempCelcius = new double[] {leader.getTemperature(), follower.getTemperature()};
    }

    @Override
    public void setHeight(double heightMeters) {
        double ffSign = 1.0;
        if (getHeight() > SECOND_STAGE_HEIGHT) {
            leader.set(ControlMode.MotionMagic, metersToIntegratedSensorUnits(heightMeters), DemandType.ArbitraryFeedForward, ffSign * (GRAVITY_FF + CF_SPRING_FF));
        } else {
            leader.set(ControlMode.MotionMagic, metersToIntegratedSensorUnits(heightMeters), DemandType.ArbitraryFeedForward, ffSign * GRAVITY_FF);
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

    @Override
    public void updateTunableNumbers() {
        if (kP.hasChanged()) {
            leader.config_kP(0, integratedSensorUnitsToMetersPerSecond(kP.get()) * 1023.0);
        }

        if (kD.hasChanged()) {
            leader.config_kD(0, integratedSensorUnitsToMetersPerSecond(kD.get()) * 1023.0);
        }

        if (kF.hasChanged()) {
            leader.config_kD(0, 1023.0/metersPerSecondToIntegratedSensorUnits(kF.get()));
        }

        if (kMaxVelocity.hasChanged()) {
            leader.configMotionCruiseVelocity(metersPerSecondToIntegratedSensorUnits(kMaxVelocity.get()));
        }

        if (kMaxAccel.hasChanged()) {
            System.out.println("I've changed!");
            leader.configMotionAcceleration(metersPerSecondToIntegratedSensorUnits(kMaxAccel.get()));
        }
    }

    /* converts integrated sensor units to meters */
    private double integratedSensorUnitsToMeters(double integratedSensorUnits) {
        return integratedSensorUnits * ((ELEVATOR_GEARING * Math.PI * ELEVATOR_PULLEY_PITCH_DIAMETER)/2048.0);
    }

    /* converts meters to integrated sensor units */
    private double metersToIntegratedSensorUnits(double meters) {
        return meters * (2048.0/(ELEVATOR_GEARING * Math.PI * ELEVATOR_PULLEY_PITCH_DIAMETER));
    }   

    /* converts integrated sensor units to meters per second */
    private double integratedSensorUnitsToMetersPerSecond(double integratedSensorUnits) {
        return integratedSensorUnits * ((ELEVATOR_GEARING * (600.0/2048.0) * Math.PI * ELEVATOR_PULLEY_PITCH_DIAMETER)/60.0);
    }

    /* converts meters per second to integrated sensor units */
    private double metersPerSecondToIntegratedSensorUnits(double metersPerSecond) {
        return metersPerSecond * (60.0/(ELEVATOR_GEARING * (600.0/2048.0) * Math.PI * ELEVATOR_PULLEY_PITCH_DIAMETER));
    }

    /* returns the height of the climber in meters */
    private double getHeight() {
        return integratedSensorUnitsToMeters(leader.getSelectedSensorPosition());
    }
    
}

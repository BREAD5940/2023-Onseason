package frc.robot.subsystems.elevatorarm;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.MathUtil;
import frc.robot.commons.LoggedTunableNumber;

import static frc.robot.Constants.Elevator.*;
import static frc.robot.Constants.Electrical.*;

public class ElevatorIOTalonFX implements ElevatorIO {

    TalonFX leader = new TalonFX(ELEVATOR_LEFT_ID, CANIVORE_BUS_NAME);
    TalonFX follower = new TalonFX(ELEVATOR_RIGHT_ID, CANIVORE_BUS_NAME);

    LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 0.5);
    LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0.075);
    LoggedTunableNumber kForwardsKa = new LoggedTunableNumber("Elevator/kForwardskA", 0.01);
    LoggedTunableNumber kBackwardsKa = new LoggedTunableNumber("Elevator/kBackwardskA", 0.005);
    LoggedTunableNumber kMaxVelocity = new LoggedTunableNumber("Elevator/kMaxVelocity", 3.435000);
    LoggedTunableNumber kMotionCruiseVelocity = new LoggedTunableNumber("Elevator/kMotionCruiseVelocity", 3.0); 
    LoggedTunableNumber kMaxAccel = new LoggedTunableNumber("Elevator/kMaxAccel", 12.0); 

    double lastVelocityTarget = 0.0;
    double mLastCommandedPosition = 0.0;

    public ElevatorIOTalonFX() {
        /* configurations for the leader motor */
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.slot0.kP = integratedSensorUnitsToMeters(kP.get()) * 1023.0;
        leaderConfig.slot0.kI = integratedSensorUnitsToMeters(0) * 1023.0;
        leaderConfig.slot0.kD = integratedSensorUnitsToMetersPerSecond(kD.get()) * 1023.0;
        leaderConfig.slot0.kF = 1023.0/metersPerSecondToIntegratedSensorUnits(kMaxVelocity.get());
        leaderConfig.motionCruiseVelocity = metersPerSecondToIntegratedSensorUnits(kMotionCruiseVelocity.get());
        leaderConfig.motionAcceleration = metersPerSecondToIntegratedSensorUnits(kMaxAccel.get());
        leaderConfig.voltageCompSaturation = 10.5;
        leaderConfig.neutralDeadband = 0.001;
        leaderConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 100, 100, 1.5);
        leader.setInverted(ELEVATOR_LEFT_INVERT_TYPE);
        leader.setNeutralMode(NeutralMode.Coast);
        leader.enableVoltageCompensation(true);
        leader.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        leader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
        leader.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 5);
        leader.configAllSettings(leaderConfig);
        leader.setSelectedSensorPosition(0.0);

        /* configurations for the follower motor */
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.neutralDeadband = 0.001;
        followerConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 100, 100, 1.5);
        follower.configAllSettings(followerConfig);
        follower.follow(leader);
        follower.setInverted(ELEVATOR_RIGHT_INVERT_TYPE);
        follower.setStatusFramePeriod(StatusFrame.Status_1_General, 197);
        follower.setNeutralMode(NeutralMode.Coast);
        follower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 193);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.posMeters = getHeight();
        inputs.velMetersPerSecond = integratedSensorUnitsToMetersPerSecond(leader.getSelectedSensorVelocity());
        inputs.velTarget = integratedSensorUnitsToMetersPerSecond(leader.getActiveTrajectoryVelocity());
        inputs.posTarget = integratedSensorUnitsToMeters(leader.getActiveTrajectoryPosition());
        inputs.appliedVoltage = leader.getMotorOutputVoltage();
        inputs.currentAmps = new double[] {leader.getStatorCurrent(), follower.getStatorCurrent()};
        inputs.tempCelcius = new double[] {leader.getTemperature(), follower.getTemperature()};

        inputs.lastFollowerError = follower.getLastError().toString();
        inputs.lastLeaderError = leader.getLastError().toString();
    }

    @Override
    public void setHeight(double heightMeters, boolean goSlow) {
        if (mLastCommandedPosition != heightMeters) {
            mLastCommandedPosition = heightMeters;
            if (Math.abs(getHeight() - heightMeters) < 0.25) {
                leader.configMotionAcceleration(metersPerSecondToIntegratedSensorUnits(3.0));
                leader.configMotionCruiseVelocity(metersPerSecondToIntegratedSensorUnits(1.5));
            } else if (goSlow) {
                leader.configMotionAcceleration(metersPerSecondToIntegratedSensorUnits(6.0));
                leader.configMotionCruiseVelocity(metersPerSecondToIntegratedSensorUnits(3.0));
            } else {
                leader.configMotionAcceleration(metersPerSecondToIntegratedSensorUnits(12.0));
                leader.configMotionCruiseVelocity(metersPerSecondToIntegratedSensorUnits(3.0));
            }
        }
        double currentVelocityTarget = integratedSensorUnitsToMetersPerSecond(leader.getActiveTrajectoryVelocity());
        double elevatorKA = 0.0;
        if (currentVelocityTarget > lastVelocityTarget) {
            elevatorKA = kMaxAccel.get() * kForwardsKa.get();
        } else if (currentVelocityTarget < lastVelocityTarget) {
            elevatorKA = kMaxAccel.get() * -kBackwardsKa.get();
        }
        lastVelocityTarget = currentVelocityTarget;
        double elevatorKG = getHeight() < SECOND_STAGE_HEIGHT ? ELEVATOR_BELOW_STAGE1_KG : ELEVATOR_ABOVE_STAGE1_KG;
        double elevatorArbFF = MathUtil.clamp(elevatorKA + elevatorKG, -0.999, 0.999);
        leader.set(ControlMode.MotionMagic, metersToIntegratedSensorUnits(heightMeters), DemandType.ArbitraryFeedForward, elevatorArbFF);
        Logger.getInstance().recordOutput("Elevator/ArbFF", elevatorArbFF);
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
        if (kP.hasChanged(0)) {
            leader.config_kP(0, integratedSensorUnitsToMetersPerSecond(kP.get()) * 1023.0);
        }

        if (kD.hasChanged(0)) {
            leader.config_kD(0, integratedSensorUnitsToMetersPerSecond(kD.get()) * 1023.0);
        }

        if (kMaxVelocity.hasChanged(0)) {
            leader.config_kF(0, 1023.0/metersPerSecondToIntegratedSensorUnits(kMaxVelocity.get()));
        }

        if (kMaxVelocity.hasChanged(0)) {
            leader.configMotionCruiseVelocity(metersPerSecondToIntegratedSensorUnits(kMaxVelocity.get()));
        }

        if (kMaxAccel.hasChanged(0)) {
            leader.configMotionAcceleration(metersPerSecondToIntegratedSensorUnits(kMaxAccel.get()));
        }
    }

    /** converts integrated sensor units to meters */
    private double integratedSensorUnitsToMeters(double integratedSensorUnits) {
        return integratedSensorUnits * ((ELEVATOR_GEARING * Math.PI * ELEVATOR_PULLEY_PITCH_DIAMETER)/2048.0);
    }

    /** converts meters to integrated sensor units */
    private double metersToIntegratedSensorUnits(double meters) {
        return meters * (2048.0/(ELEVATOR_GEARING * Math.PI * ELEVATOR_PULLEY_PITCH_DIAMETER));
    }   

    /** converts integrated sensor units to meters per second */
    private double integratedSensorUnitsToMetersPerSecond(double integratedSensorUnits) {
        return integratedSensorUnits * ((ELEVATOR_GEARING * (600.0/2048.0) * Math.PI * ELEVATOR_PULLEY_PITCH_DIAMETER)/60.0);
    }

    /** converts meters per second to integrated sensor units */
    private double metersPerSecondToIntegratedSensorUnits(double metersPerSecond) {
        return metersPerSecond * (60.0/(ELEVATOR_GEARING * (600.0/2048.0) * Math.PI * ELEVATOR_PULLEY_PITCH_DIAMETER));
    }

    /** returns the height of the climber in meters */
    private double getHeight() {
        return integratedSensorUnitsToMeters(leader.getSelectedSensorPosition());
    }
    
    /** resets sticky faults to allow error to change from anything back to "ok" */
    public void clearFault(){
        leader.clearStickyFaults();
        follower.clearStickyFaults();
    }
}

package frc.robot.subsystems.floorintake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.commons.LoggedTunableNumber;

import static frc.robot.Constants.FloorIntake.*;
import static frc.robot.Constants.Electrical.*;

public class FloorIntakeIOTalonFX implements FloorIntakeIO {
    
    TalonFX deploy = new TalonFX(DEPLOY_ID, CANIVORE_BUS_NAME);
    TalonFX roller = new TalonFX(ROLLER_ID, CANIVORE_BUS_NAME);

    private double mCurrentLimit = 0.0;
    private double mCurrentLimitTriggerThreshhold = 0.0;
    private double mcurrentLimitThresholdTime = 0.0;
    
    LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", FLOOR_INTAKE_KP);
    LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", FLOOR_INTAKE_KD);
    LoggedTunableNumber kF = new LoggedTunableNumber("Elevator/kF", FLOOR_INTAKE_KF);
    LoggedTunableNumber kMaxVelocity = new LoggedTunableNumber("Elevator/kMaxVelocity", 2.1); 
    LoggedTunableNumber kMaxAccel = new LoggedTunableNumber("Elevator/kMaxAccel", 6.0); 

    public FloorIntakeIOTalonFX() {
        /* configurations for the leader motor */
        TalonFXConfiguration deployConfig = new TalonFXConfiguration();
        deployConfig.slot0.kP = degreesToIntegratedSensorUnits(0.0000001) * 1023.0;
        deployConfig.slot0.kI = degreesToIntegratedSensorUnits(0) * 1023.0;
        deployConfig.slot0.kD = degreesToIntegratedSensorUnits(0.0) * 1023.0;
        deployConfig.slot0.kF = 1023.0/degreesPerSecondToIntegratedSensorUnits(DEPLOY_MAX_SPEED) * 1.36 * 50.0/55.8; // 1.36 is an empiracally obtained constant that adjusts the feedforward so that the intake can track its motion profile well without feedback
        deployConfig.slot1.kP = degreesPerSecondToIntegratedSensorUnits(0.01);
        deployConfig.slot1.kI = degreesPerSecondToIntegratedSensorUnits(0.0);
        deployConfig.slot1.kD = degreesPerSecondToIntegratedSensorUnits(0.0);
        deployConfig.slot1.kF = 1023.0/degreesPerSecondToIntegratedSensorUnits(DEPLOY_MAX_SPEED) * 1.36 * 50.0/55.8;
        deployConfig.motionCruiseVelocity = degreesPerSecondToIntegratedSensorUnits(360.0);
        deployConfig.motionAcceleration = degreesPerSecondToIntegratedSensorUnits(1000.0);
        deployConfig.voltageCompSaturation = 10.5;
        deployConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 30.0, 40.0, 1.5);
        deployConfig.neutralDeadband = 0.001;
        deploy.setInverted(DEPLOY_INVERT_TYPE);
        deploy.setNeutralMode(NeutralMode.Brake);
        deploy.enableVoltageCompensation(true);
        deploy.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        deploy.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
        deploy.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10);
        deploy.configAllSettings(deployConfig);
        deploy.setSelectedSensorPosition(0.0);

        roller.setInverted(ROLLER_INVERT_TYPE);
        roller.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50.0, 60.0, 1.5));
    }

    @Override
    public void updateInputs(FloorIntakeIOInputs inputs) {
        inputs.rollerCurrentAmps = roller.getStatorCurrent();
        inputs.rollerAppliedVoltage = roller.getMotorOutputVoltage();
        inputs.rollerTempCelcius = roller.getTemperature();
        inputs.deployVelDegreesPerSecond = integratedSensorUnitsToDegreesPerSecond(deploy.getSelectedSensorVelocity());
        inputs.deployAngleDegrees = getAngle();
        inputs.deployCurrentAmps = deploy.getStatorCurrent();
        inputs.deployAppliedVoltage = deploy.getMotorOutputVoltage();
        inputs.deployTempCelcius = deploy.getTemperature();
        inputs.deployPositionTarget = integratedSensorUnitsToDegrees(deploy.getActiveTrajectoryPosition());
        inputs.deployVelocityTarget = integratedSensorUnitsToDegreesPerSecond(deploy.getActiveTrajectoryVelocity());
        inputs.deployDutyCycle = deploy.getMotorOutputPercent();

        inputs.lastDeployError = deploy.getLastError().toString();
        inputs.lastRollerError = deploy.getLastError().toString();
    }

    @Override
    public void setRollerPercent(double percent) {
        roller.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void setDeployAngle(double angle) {
        deploy.selectProfileSlot(0, 0);
        double arbFF = Math.sin(Units.degreesToRadians(getAngle() - 30.0)) * -0.025; // -0.025 is an empirically obtained gravity feedforward to offset gravity (multiplied by the angle of the intake to increase/decrease the factor depending on where the intake is)
        angle = MathUtil.clamp(angle, INTAKE_MIN_POSITION, INTAKE_MAX_POSITION);
        deploy.set(ControlMode.MotionMagic, degreesToIntegratedSensorUnits(angle), DemandType.ArbitraryFeedForward, arbFF);
    }
    @Override
    public void setDeployVelocity(double velocityDegreesPerSecond) {
        deploy.selectProfileSlot(1, 0);
        double arbFF = Math.sin(Units.degreesToRadians(getAngle() - 30.0)) * 0.025; // -0.025 is an empirically obtained gravity feedforward to offset gravity (multiplied by the angle of the intake to increase/decrease the factor depending on where the intake is)
        deploy.set(ControlMode.Velocity, degreesPerSecondToIntegratedSensorUnits(velocityDegreesPerSecond), DemandType.ArbitraryFeedForward, arbFF);
    }

    @Override
    public void setDeployPercent(double percent) {
        deploy.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void setCurrentLimit(double currentLimit, double currentLimitTriggerThreshold, double currentLimitThresholdTime) {
        if (currentLimit != mCurrentLimit || currentLimitTriggerThreshold != mCurrentLimitTriggerThreshhold || currentLimitThresholdTime != mcurrentLimitThresholdTime) {
            mCurrentLimit = currentLimit;
            mCurrentLimitTriggerThreshhold = currentLimitTriggerThreshold;
            mcurrentLimitThresholdTime = currentLimitThresholdTime;
            deploy.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit, currentLimitTriggerThreshold, currentLimitThresholdTime));
        }
    }

    @Override
    public void enableDeployBrakeMode(boolean enable) {
        deploy.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public void resetDeployAngle(double newAngle) {
        deploy.setSelectedSensorPosition(degreesToIntegratedSensorUnits(newAngle));
    }

    @Override
    public void enableRollerBrakeMode(boolean enable) {
        roller.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /* converts integrated sensor units to meters */
    private static double integratedSensorUnitsToDegrees(double integratedSensorUnits) {
        return integratedSensorUnits * ((DEPLOY_GEAR_RATIO * 360.0)/2048.0);
    }

    /* converts meters to integrated sensor units */
    private static double degreesToIntegratedSensorUnits(double degrees) {
        return degrees * (2048.0/(DEPLOY_GEAR_RATIO * 360.0));
    }

    /* converts integrated sensor units to meters per second */
    private static double integratedSensorUnitsToDegreesPerSecond(double integratedSensorUnits) {
        return integratedSensorUnits * ((DEPLOY_GEAR_RATIO * (600.0/2048.0) * 360.0)/60.0);
    }

    /* converts meters per second to integrated sensor units */
    private static double degreesPerSecondToIntegratedSensorUnits(double degreesPerSecond) {
        return degreesPerSecond * (60.0/(DEPLOY_GEAR_RATIO * (600.0/2048.0) * 360.0));
    }

    /* Returns the angle of the intake */
    private double getAngle() {
        return integratedSensorUnitsToDegrees(deploy.getSelectedSensorPosition());
    }

    /* resets sticky faults to allow error to change from anything back to "ok" */
    public void clearFault(){
        deploy.clearStickyFaults();
        roller.clearStickyFaults();
    }
}

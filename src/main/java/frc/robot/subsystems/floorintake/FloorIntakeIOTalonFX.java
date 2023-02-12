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

import static frc.robot.Constants.FloorIntake.*;
import static frc.robot.Constants.Electrical.*;

public class FloorIntakeIOTalonFX implements FloorIntakeIO {
    
    TalonFX deploy = new TalonFX(DEPLOY_ID, CANIVORE_BUS_NAME);
    TalonFX roller = new TalonFX(ROLLER_ID, CANIVORE_BUS_NAME);

    public FloorIntakeIOTalonFX() {
        /* configurations for the leader motor */
        TalonFXConfiguration deployConfig = new TalonFXConfiguration();
        deployConfig.slot0.kP = degreesToIntegratedSensorUnits(0.0000001) * 1023.0;
        deployConfig.slot0.kI = degreesToIntegratedSensorUnits(0) * 1023.0;
        deployConfig.slot0.kD = degreesToIntegratedSensorUnits(0.0) * 1023.0;
        deployConfig.slot0.kF = 1023.0/degreesPerSecondToIntegratedSensorUnits(DEPLOY_MAX_SPEED) * 1.36; // 1.36 is an empiracally obtained constant that adjusts the feedforward so that the intake can track its motion profile well without feedback
        deployConfig.motionCruiseVelocity = degreesPerSecondToIntegratedSensorUnits(300.0);
        deployConfig.motionAcceleration = degreesPerSecondToIntegratedSensorUnits(600.0);
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
    }

    @Override
    public void setRollerPercent(double percent) {
        roller.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void setDeployAngle(double angle) {
        double arbFF = Math.sin(Units.degreesToRadians(getAngle() - 30.0)) * -0.03; // -0.025 is an empirically obtained gravity feedforward to offset gravity (multiplied by the angle of the intake to increase/decrease the factor depending on where the intake is)
        angle = MathUtil.clamp(angle, INTAKE_MIN_POSITION, INTAKE_MAX_POSITION);
        deploy.set(ControlMode.MotionMagic, degreesToIntegratedSensorUnits(angle), DemandType.ArbitraryFeedForward, arbFF);
    }

    @Override
    public void setDeployPercent(double percent) {
        deploy.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void setCurrentLimit(double currentLimit, double currentLimitTriggerThreshold, double currentLimitThresholdTime) {
        deploy.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, currentLimit, currentLimitTriggerThreshold, currentLimitThresholdTime));
    }

    @Override
    public void enableDeployBrakeMode(boolean enable) {
        deploy.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public void resetDeployAngle(double newAngle) {
        deploy.setSelectedSensorPosition(degreesPerSecondToIntegratedSensorUnits(newAngle));
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
}

package frc.robot.subsystems.floorintake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import static frc.robot.Constants.FloorIntake.*;
import static frc.robot.Constants.Electrical.*;

public class FloorIntakeIOTalonFX implements FloorIntakeIO {
    
    TalonFX deploy = new TalonFX(DEPLOY_ID, CANIVORE_BUS_NAME);
    TalonFX roller = new TalonFX(ROLLER_ID, CANIVORE_BUS_NAME);

    public FloorIntakeIOTalonFX() {
        /* configurations for the leader motor */
        TalonFXConfiguration deployConfig = new TalonFXConfiguration();
        deployConfig.slot0.kP = integratedSensorUnitsToDegreesPerSecond(1.0) * 1023.0;
        deployConfig.slot0.kI = integratedSensorUnitsToDegreesPerSecond(0) * 1023.0;
        deployConfig.slot0.kD = integratedSensorUnitsToDegreesPerSecond(0) * 1023.0;
        deployConfig.slot0.kF = 1023.0/degreesPerSecondToIntegratedSensorUnits(DEPLOY_MAX_SPEED);
        deployConfig.motionCruiseVelocity = degreesPerSecondToIntegratedSensorUnits(30);
        deployConfig.motionAcceleration = degreesPerSecondToIntegratedSensorUnits(30);
        deployConfig.voltageCompSaturation = 10.5;
        deploy.setInverted(DEPLOY_INVERT_TYPE);
        deploy.enableVoltageCompensation(true);
        deploy.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        deploy.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
        deploy.configAllSettings(deployConfig);
        deploy.setSelectedSensorPosition(0.0);

        roller.setInverted(ROLLER_INVERT_TYPE);
    }

    @Override
    public void updateInputs(FloorIntakeIOInputs inputs) {
        inputs.rollerCurrentAmps = roller.getStatorCurrent();
        inputs.rollerAppliedVoltage = roller.getMotorOutputVoltage();
        inputs.rollerTempCelcius = roller.getTemperature();
        inputs.deployVelDegreesPerSecond = integratedSensorUnitsToDegreesPerSecond(deploy.getSelectedSensorVelocity());
        inputs.deployAngleDegrees = integratedSensorUnitsToDegrees(deploy.getSelectedSensorPosition());
        inputs.deployCurrentAmps = deploy.getStatorCurrent();
        inputs.deployAppliedVoltage = deploy.getMotorOutputVoltage();
        inputs.deployTempCelcius = deploy.getTemperature();
    }

    @Override
    public void setRollerPercent(double percent) {
        roller.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void setDeployAngle(double angle) {
       deploy.set(ControlMode.Position, angle);
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
    public void enableRollerBrakeMode(boolean enable) {
        roller.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /* converts integrated sensor units to meters */
    private double integratedSensorUnitsToDegrees(double integratedSensorUnits) {
        return integratedSensorUnits * ((DEPLOY_GEAR_RATIO * 360.0)/2048.0);
    }

    /* converts meters to integrated sensor units */
    private double degreesToIntegratedSensorUnits(double degrees) {
        return degrees * (2048.0/(DEPLOY_GEAR_RATIO * 360.0));
    }

    /* converts integrated sensor units to meters per second */
    private double integratedSensorUnitsToDegreesPerSecond(double integratedSensorUnits) {
        return integratedSensorUnits * ((DEPLOY_GEAR_RATIO * (600.0/2048.0) * 360.0)/60.0);
    }

    /* converts meters per second to integrated sensor units */
    private double degreesPerSecondToIntegratedSensorUnits(double degreesPerSecond) {
        return degreesPerSecond * (60.0/(DEPLOY_GEAR_RATIO * (600.0/2048.0) * 360.0));
    }
}

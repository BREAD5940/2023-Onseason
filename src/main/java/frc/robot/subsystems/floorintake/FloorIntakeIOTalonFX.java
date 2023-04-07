package frc.robot.subsystems.floorintake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Drive;
import frc.robot.commons.Conversions;
import frc.robot.commons.LoggedTunableNumber;

import static frc.robot.Constants.FloorIntake.*;
import static frc.robot.Constants.Electrical.*;
import static frc.robot.Constants.FaultChecker.*;

public class FloorIntakeIOTalonFX implements FloorIntakeIO {
    
    TalonFX deploy = new TalonFX(DEPLOY_ID, CANIVORE_BUS_NAME);
    TalonFX roller = new TalonFX(ROLLER_ID, CANIVORE_BUS_NAME);
    CANCoder floorIntakeAzimuth;

    private double mCurrentLimit = 0.0;
    private double mCurrentLimitTriggerThreshhold = 0.0;
    private double mcurrentLimitThresholdTime = 0.0;

    private int moterErrorWaitI = 0;
    
    LoggedTunableNumber kP = new LoggedTunableNumber("FloorIntake/kP", FLOOR_INTAKE_KP);
    LoggedTunableNumber kD = new LoggedTunableNumber("FloorIntake/kD", FLOOR_INTAKE_KD);
    LoggedTunableNumber kF = new LoggedTunableNumber("FloorIntake/kF", FLOOR_INTAKE_KF);
    LoggedTunableNumber kMaxVelocity = new LoggedTunableNumber("FloorIntake/kMaxVelocity", DEPLOY_MAX_SPEED); 
    LoggedTunableNumber kMaxAccel = new LoggedTunableNumber("FloorIntake/kMaxAccel", 1000.0); 
    LoggedTunableNumber kMotionCruiseVelocity = new LoggedTunableNumber("FloorIntake/kMotionCruiseVelocity", 360.0); 

    public FloorIntakeIOTalonFX() {
        /* configuations for the floor intake azimuth */
        floorIntakeAzimuth = new CANCoder(FLOOR_INTAKE_AZIMUTH_ID, CANIVORE_BUS_NAME);
        floorIntakeAzimuth .configSensorDirection(FLOOR_INTAKE_AZIMUTH_INVERTED);
        floorIntakeAzimuth.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        floorIntakeAzimuth.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        /* configurations for the leader motor */
        TalonFXConfiguration deployConfig = new TalonFXConfiguration();
        deployConfig.remoteFilter0.remoteSensorDeviceID = floorIntakeAzimuth.getDeviceID();
        deployConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        deployConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        deployConfig.slot0.kP = degreesToCANCoderSensorUnits(kP.get()) * 1023.0;
        deployConfig.slot0.kI = degreesToCANCoderSensorUnits(0.0) * 1023.0;
        deployConfig.slot0.kD = degreesToCANCoderSensorUnits(kD.get()) * 1023.0;
        deployConfig.slot0.kF = 1023.0/degreesPerSecondToCANCoderSensorUnits(DEPLOY_MAX_SPEED) * 1.36 * 50.0/55.8; // 1.36 is an empiracally obtained constant that adjusts the feedforward so that the intake can track its motion profile well without feedback
        deployConfig.motionCruiseVelocity = degreesPerSecondToCANCoderSensorUnits(360.0);
        deployConfig.motionAcceleration = degreesPerSecondToCANCoderSensorUnits(1000.0);
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
        deploy.setSensorPhase(FLOOR_INTAKE_PHASE_INVERTED);

        roller.setInverted(ROLLER_INVERT_TYPE);
        roller.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50.0, 60.0, 1.5));
    }

    @Override
    public void updateInputs(FloorIntakeIOInputs inputs) {
        inputs.rollerCurrentAmps = roller.getStatorCurrent();
        inputs.rollerAppliedVoltage = roller.getMotorOutputVoltage();
        inputs.rollerTempCelcius = roller.getTemperature();
        inputs.rollerMotorSpeedRPM = Conversions.falconToRPM(roller.getSelectedSensorVelocity(), 1.0);
        inputs.angleDegrees = getAngle();
        inputs.angleDegreesCC = floorIntakeAzimuth.getPosition();
        inputs.velDegreesPerSecond = CANCoderSensorUnitsToDegreesPerSecond(deploy.getSelectedSensorVelocity());
        inputs.velDegreesPerSecondCC = floorIntakeAzimuth.getVelocity();
        inputs.deployCurrentAmps = deploy.getStatorCurrent();
        inputs.deployAppliedVoltage = deploy.getMotorOutputVoltage();
        inputs.deployTempCelcius = deploy.getTemperature();
        inputs.deployPositionTarget = CANCoderSensorUnitsToDegrees(deploy.getActiveTrajectoryPosition());
        inputs.deployVelocityTarget = CANCoderSensorUnitsToDegreesPerSecond(deploy.getActiveTrajectoryVelocity());
        inputs.deployDutyCycle = deploy.getMotorOutputPercent();

        moterErrorWaitI++;
		if (moterErrorWaitI >= LOOPS_PER_ERROR_CHECK) {
			moterErrorWaitI = 0;
        	inputs.lastDeployError = deploy.getLastError().toString();
        	inputs.lastRollerError = roller.getLastError().toString();
            clearFault();
		}
    }

    @Override
    public void setRollerPercent(double percent) {
        roller.set(ControlMode.PercentOutput, percent);
    }

	@Override
    public void setDeployPercent(double percent) {
        deploy.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void setDeployAngle(double angle) {
        deploy.selectProfileSlot(0, 0);
        double arbFF = Math.sin(Units.degreesToRadians(getAngle() - 30.0)) * -0.025; // -0.025 is an empirically obtained gravity feedforward to offset gravity (multiplied by the angle of the intake to increase/decrease the factor depending on where the intake is)
        angle = MathUtil.clamp(angle, INTAKE_MIN_POSITION, INTAKE_MAX_POSITION);
        deploy.set(ControlMode.MotionMagic, degreesToCANCoderSensorUnits(angle), DemandType.ArbitraryFeedForward, arbFF);
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
    public void resetAngle() {
        double angle = floorIntakeAzimuth.getAbsolutePosition() - FLOOR_INTAKE_AZIMUTH_DEGREE_OFFSET;
        /* puts the angle in the [0, 360] range */
        while (angle < 0.0) {
            angle = 360.0 + angle;
        }
    
        /* if the angle is in our null range, then we subtract 360 from it */
        while (angle > FLOOR_INTAKE_NULL_RANGE) {
            angle -= 360.0;
        }
        floorIntakeAzimuth.setPosition(angle);
    }

    @Override
    public void enableRollerBrakeMode(boolean enable) {
        roller.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public void updateTunableNumbers() {
        if (kP.hasChanged(0)) {
            deploy.config_kP(0, degreesToCANCoderSensorUnits(kP.get()) * 1023.0);
        }

        if (kD.hasChanged(0)) {
            deploy.config_kD(0, degreesToCANCoderSensorUnits(kD.get()) * 1023.0);
        }

        if (kMaxVelocity.hasChanged(0)) {
            deploy.config_kF(0, 1023.0/degreesPerSecondToCANCoderSensorUnits(kMaxVelocity.get()));
        }

        if (kMaxVelocity.hasChanged(0)) {
            deploy.configMotionCruiseVelocity(degreesPerSecondToCANCoderSensorUnits(kMaxVelocity.get()));
        }

        if (kMaxAccel.hasChanged(0)) {
            deploy.configMotionAcceleration(degreesPerSecondToCANCoderSensorUnits(kMaxAccel.get()));
        }
    }

    private static double CANCoderSensorUnitsToDegrees(double sensorUnits) {
        return sensorUnits * (360.0) / 4096.0;
    }

    private static double degreesToCANCoderSensorUnits(double degrees) {
        return degrees * 4096.0 / (360.0);
    }

    private static double CANCoderSensorUnitsToDegreesPerSecond(double sensorUnits) {
        return sensorUnits * ((360.0 * 10.0)/4096.0);
    }

    private static double degreesPerSecondToCANCoderSensorUnits(double degrees) {
        return degrees * (4096.0/(360.0 * 10.0));
    }

    /* Returns the angle of the intake */
    private double getAngle() {
        return CANCoderSensorUnitsToDegrees(deploy.getSelectedSensorPosition());
    }

    /** resets sticky faults to allow error to change from anything back to "ok" */
    public void clearFault(){
        deploy.clearStickyFaults();
        roller.clearStickyFaults();
    }
 
}

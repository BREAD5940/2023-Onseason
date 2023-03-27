package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenixpro.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenixpro.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.configs.FeedbackConfigs;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.Slot1Configs;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.configs.VoltageConfigs;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.commons.Alert;
import frc.robot.commons.Conversions;
import frc.robot.commons.Alert.AlertType;

import static frc.robot.Constants.Drive.*;
import static frc.robot.Constants.Electrical.*;

public class ModuleIOTalonFX implements ModuleIO {

    private final com.ctre.phoenixpro.hardware.TalonFX drive;
    private final TalonFX steer;
    private final double offset;
    public final CANCoder azimuth;
    public double[] desiredState = {0, 0};
	private int errorGetterIndex = 0;
	private String lastSteerError;
	private String lastDriveError;
	private String lastAzimuthError;

    private DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0, true, false);
    private VelocityVoltage velocityVoltageOut = new VelocityVoltage(0.0, true, 0.0, 0, false);

    public ModuleIOTalonFX(int driveID, int steerID, int azimuthID, Rotation2d offset, InvertedValue driveDirection, TalonFXInvertType steerReversed, boolean azimuthReversed, String moduleIdentifier) {
        this.offset = offset.getDegrees();

        // Configure the driving motor
        drive = new com.ctre.phoenixpro.hardware.TalonFX(driveID, CANIVORE_BUS_NAME);
        TalonFXConfigurator configurator = drive.getConfigurator();

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = integratedSensorUnitsToWheelSpeedMetersPerSecond(0.35) * 10.0;
        slot0Configs.kI = integratedSensorUnitsToWheelSpeedMetersPerSecond(0.0) * 10.0;
        slot0Configs.kD = integratedSensorUnitsToWheelSpeedMetersPerSecond(0.0) * 10.0;
        slot0Configs.kS = 0.6;
        slot0Configs.kV = 10.7/wheelSpeedMetersPerSecondToIntegratedSensorUnits(ROBOT_MAX_SPEED);

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.Inverted = driveDirection;
        motorOutputConfigs.PeakForwardDutyCycle = 1.0;
        motorOutputConfigs.PeakReverseDutyCycle = -1.0;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        currentLimitsConfigs.StatorCurrentLimit = 120.0;

        configurator.apply(feedbackConfigs);
        configurator.apply(slot0Configs);
        configurator.apply(motorOutputConfigs);
        configurator.apply(currentLimitsConfigs);

        drive.setRotorPosition(0.0);

        // Create CAN Coder object
        azimuth = new CANCoder(azimuthID, CANIVORE_BUS_NAME);
        azimuth.configMagnetOffset(0.0); 
        azimuth.configSensorDirection(false);
        azimuth.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        azimuth.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        // Configure the steering motor
        steer = new TalonFX(steerID, CANIVORE_BUS_NAME);
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        steerConfig.slot0.kP = 0.3;
        steerConfig.slot0.kI = CANCoderSensorUnitsToRadians(0.0) * 1023.0;
        steerConfig.slot0.kD = CANCoderSensorUnitsToRadians(0.0) * 1023.0;
        steerConfig.slot0.closedLoopPeakOutput = 1.0;
        steerConfig.peakOutputForward = 1.0;
        steerConfig.peakOutputReverse = -1.0;
        steerConfig.neutralDeadband = 0.0001;
        steerConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 50.0, 50.0, 1.5);
        steer.setInverted(steerReversed);
        steer.setNeutralMode(NeutralMode.Coast);
        steer.setSensorPhase(true);
        steer.set(ControlMode.Velocity, 0.0);
        steer.configAllSettings(steerConfig);
        steer.selectProfileSlot(0, 0);
        steer.setStatusFramePeriod(StatusFrame.Status_1_General, 99);
        steer.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 15);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveVelocityMetersPerSec = integratedSensorUnitsToWheelSpeedMetersPerSecond(drive.getRotorVelocity().getValue());
        inputs.driveAppliedVolts = drive.getDutyCycle().getValue() * drive.getSupplyVoltage().getValue();
        inputs.driveCurrentAmps = drive.getStatorCurrent().getValue();
        inputs.driveTempCelcius = drive.getDeviceTemp().getValue();
        inputs.driveDistanceMeters = integratedSensorUnitsToWheelPositionMeters(drive.getRotorPosition().getValue());
        inputs.driveOutputPercent = drive.get();
        inputs.rawDriveRPM = drive.getRotorVelocity().getValue();

        inputs.moduleAngleRads = Units.degreesToRadians(Conversions.falconToDegrees(steer.getSelectedSensorPosition(), STEER_GEARING));
        inputs.rawAbsolutePositionDegrees = azimuth.getAbsolutePosition();
        inputs.turnAppliedVolts = steer.getMotorOutputVoltage();
        inputs.turnCurrentAmps = steer.getStatorCurrent();
        inputs.turnTempCelcius = steer.getTemperature();
		errorGetterIndex ++;
		if (errorGetterIndex >= 50) {
			errorGetterIndex = 0;
			inputs.lastSteerError = steer.getLastError().toString();
			lastSteerError = inputs.lastSteerError;
			inputs.lastDriveError = drive.getLastError().toString();
			lastDriveError = inputs.lastDriveError;
			inputs.lastAzimuthError = azimuth.getLastError().toString();
			lastAzimuthError = inputs.lastAzimuthError;
		}
    }

    @Override
    public void setDriveVelocity(double velocityMetersPerSecond, boolean auto) {
        velocityVoltageOut.Velocity = wheelSpeedMetersPerSecondToIntegratedSensorUnits(velocityMetersPerSecond);
        drive.setControl(velocityVoltageOut);
    }

    @Override
    public void setDrivePercent(double percent) {
        dutyCycleOut.Output = percent;
        drive.setControl(dutyCycleOut);
    }

    @Override
    public void setTurnAngle(double angleDeg) {
        steer.set(TalonFXControlMode.Position, Conversions.degreesToFalcon(angleDeg, STEER_GEARING));
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        // drive.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        steer.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoderAbsolutePosition().getDegrees() - offset, STEER_GEARING);
        steer.setSelectedSensorPosition(absolutePosition);
        if (steer.getSelectedSensorPosition() == 0.0) {
            new Alert("Steer motor with id " + steer.getDeviceID() + "did not zero properly. Please restart robot or align it properly before match begins.", AlertType.ERROR);
        }
    }

    /** Converts CANCoder sensor units to radians */
    private final double CANCoderSensorUnitsToRadians(double sensorUnits) {
        return sensorUnits * (2.0 * Math.PI)/CANCODER_RESOLUTION;
    }

    /** Converts radians to CANCoder sensor units */
    private final double radiansToCANCoderSensorUnits(double radians) {
        return radians * CANCODER_RESOLUTION/(2.0 * Math.PI);
    }

    /** Converts integrated sensor units to wheel speed meters per second */
    private final double integratedSensorUnitsToWheelSpeedMetersPerSecond(double integratedSensorUnits) {
        return (DRIVE_GEARING * integratedSensorUnits * 2.0 * Math.PI * WHEEL_RADIUS);
    }

    /** Converts wheel speed meters per second to integrated sensor units */
    private final double wheelSpeedMetersPerSecondToIntegratedSensorUnits(double wheelSpeed) {
        return wheelSpeed/(DRIVE_GEARING * 2.0 * Math.PI * WHEEL_RADIUS);
    }

    /** Converts integrated sensor units to wheel speed meters per second */
    private final double integratedSensorUnitsToWheelPositionMeters(double integratedSensorUnits) {
        return integratedSensorUnitsToWheelSpeedMetersPerSecond(integratedSensorUnits);
    }

    /** Converts wheel speed meters per second to integrated sensor units */
    private final double wheelPositionMetersToIntegratedSensorUnits(double wheelSpeed) {
        return wheelSpeedMetersPerSecondToIntegratedSensorUnits(wheelSpeed);
    }  

    /** Returns a rotation2d representing the angle of the CANCoder object */
    public Rotation2d getCanCoderAbsolutePosition() {
        return Rotation2d.fromDegrees(azimuth.getAbsolutePosition());
    }

    /** resets sticky faults to allow error to change from anything back to "ok" */
    @Override
    public void clearFault(){
		if(lastAzimuthError != "OK"){
			azimuth.clearStickyFaults();
		}
		if(lastSteerError != "OK"){
			steer.clearStickyFaults();
		}
		if(lastDriveError != "OK"){
			drive.clearStickyFaults();
		} 
    }
    
}
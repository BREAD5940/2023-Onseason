package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.Drive.*;
import static frc.robot.Constants.Electrical.*;

public class ModuleIOTalonFX implements ModuleIO {

    private final TalonFX drive;
    private final TalonFX steer;
    public final CANCoder azimuth;
    public double[] desiredState = {0, 0};

     public ModuleIOTalonFX(int driveID, int steerID, int azimuthID, Rotation2d offset, TalonFXInvertType driveDirection, TalonFXInvertType steerReversed, boolean azimuthReversed, String moduleIdentifier) {
        // Configure the driving motor
        drive = new TalonFX(driveID, CANIVORE_BUS_NAME);
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        driveConfig.slot0.kP = integratedSensorUnitsToWheelSpeedMetersPerSecond(0.05) * 1023.0; // TODO check this
        driveConfig.slot0.kI = integratedSensorUnitsToWheelSpeedMetersPerSecond(0.0);
        driveConfig.slot0.kD = integratedSensorUnitsToWheelSpeedMetersPerSecond(0.0);
        driveConfig.slot0.kF = 1023.0/wheelSpeedMetersPerSecondToIntegratedSensorUnits(ROBOT_MAX_SPEED);
        driveConfig.slot1.kP = integratedSensorUnitsToWheelSpeedMetersPerSecond(0.001) * 1023.0;
        driveConfig.slot1.kI = integratedSensorUnitsToWheelSpeedMetersPerSecond(0.0);
        driveConfig.slot1.kD = integratedSensorUnitsToWheelSpeedMetersPerSecond(0.0);
        driveConfig.slot1.kF = 1023.0/wheelSpeedMetersPerSecondToIntegratedSensorUnits(ROBOT_MAX_SPEED);
        driveConfig.slot0.closedLoopPeakOutput = 1.0;
        driveConfig.peakOutputForward = 1.0;
        driveConfig.peakOutputReverse = -1.0;
        driveConfig.voltageCompSaturation = 12.0;
        driveConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 80.0, 80.0, 1.5);
        drive.setInverted(driveDirection);
        drive.setNeutralMode(NeutralMode.Brake); // TODO change back
        drive.configAllSettings(driveConfig);
        drive.set(ControlMode.Velocity, 0.0);
        drive.enableVoltageCompensation(true);
        drive.selectProfileSlot(1, 0);
        drive.setStatusFramePeriod(StatusFrame.Status_1_General, 97);
        drive.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 15);

        // Create CAN Coder object
        azimuth = new CANCoder(azimuthID, CANIVORE_BUS_NAME);
        azimuth.configMagnetOffset(offset.getDegrees());
        azimuth.configSensorDirection(false);
        azimuth.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        azimuth.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        // Configure the steering motor
        steer = new TalonFX(steerID, CANIVORE_BUS_NAME);
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.remoteFilter0.remoteSensorDeviceID = azimuth.getDeviceID();
        steerConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        steerConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        steerConfig.slot0.kP = CANCoderSensorUnitsToRadians(1.0) * 1023.0;
        steerConfig.slot0.kI = CANCoderSensorUnitsToRadians(0.0) * 1023.0;
        steerConfig.slot0.kD = CANCoderSensorUnitsToRadians(0.0) * 1023.0;
        steerConfig.slot0.closedLoopPeakOutput = 1.0;
        steerConfig.peakOutputForward = 1.0;
        steerConfig.peakOutputReverse = -1.0;
        steerConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 50.0, 50.0, 1.5);
        steer.setInverted(steerReversed);
        steer.setNeutralMode(NeutralMode.Brake);
        steer.setSensorPhase(true);
        steer.set(ControlMode.Velocity, 0.0);
        steer.configAllSettings(steerConfig);
        steer.selectProfileSlot(0, 0);
        steer.setStatusFramePeriod(StatusFrame.Status_1_General, 99);
        steer.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 15);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveVelocityMetersPerSec = (MODULE_GEARING * drive.getSelectedSensorVelocity() * (600.0/2048.0) * 2.0 * Math.PI * WHEEL_RADIUS) / 60.0;
        inputs.driveAppliedVolts = drive.getMotorOutputVoltage();
        inputs.driveCurrentAmps = drive.getStatorCurrent();
        inputs.driveTempCelcius = drive.getTemperature();
        inputs.driveDistance = (MODULE_GEARING * drive.getSelectedSensorPosition() * 2.0 * Math.PI * WHEEL_RADIUS)/2048.0;
        inputs.driveBusVoltage = drive.getBusVoltage();
        inputs.driveOutputPercent = drive.getMotorOutputPercent();

        inputs.turnAbsolutePositionRad = Units.degreesToRadians(azimuth.getPosition());
        inputs.turnAppliedVolts = steer.getMotorOutputVoltage();
        inputs.turnCurrentAmps = steer.getStatorCurrent();
        inputs.turnTempCelcius = steer.getTemperature();
    }

    @Override
    public void setDriveVelocity(double velocityMetersPerSecond) {
        drive.set(TalonFXControlMode.Velocity, wheelSpeedMetersPerSecondToIntegratedSensorUnits(velocityMetersPerSecond));
    }

    @Override
    public void setDrivePercent(double percent) {
        drive.set(TalonFXControlMode.PercentOutput, percent);
    }

    @Override
    public void setTurnAngle(double positionRads) {
        steer.set(TalonFXControlMode.Position, radiansToCANCoderSensorUnits(positionRads));
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        drive.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        drive.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
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
        return integratedSensorUnits * (MODULE_GEARING * (600.0/2048.0) * 2.0 * Math.PI * WHEEL_RADIUS) / 60.0;
    }

    /** Converts wheel speed meters per second to integrated sensor units */
    private final double wheelSpeedMetersPerSecondToIntegratedSensorUnits(double wheelSpeed) {
        return wheelSpeed * 60.0 / (MODULE_GEARING * (600.0/2048.0) * 2.0 * Math.PI * WHEEL_RADIUS);
    }  
    
}
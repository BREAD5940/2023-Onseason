package frc.robot.subsystems.superstructure;
import com.ctre.phoenix.motorcontrol.ControlMode;
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

import static frc.robot.Constants.Arm.*;
import static frc.robot.Constants.Electrical.*;

public class ArmIOTalonFX implements ArmIO {

    TalonFX arm = new TalonFX(ARM_ID);
    CANCoder armAzimuth = new CANCoder(ARM_AZIMUTH_ID);

    public ArmIOTalonFX() {
        /* configurations for the arm encoder */
        armAzimuth = new CANCoder(ARM_AZIMUTH_ID);
        armAzimuth.configSensorDirection(false);
        armAzimuth.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        armAzimuth.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        /* configurations for the arm motor */
        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        armConfig.remoteFilter0.remoteSensorDeviceID = armAzimuth.getDeviceID();
        armConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        armConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        armConfig.slot0.kP = CANCoderSensorUnitsToDegrees(1.0) * 1023.0;
        armConfig.slot0.kI = CANCoderSensorUnitsToDegrees(0) * 1023.0;
        armConfig.slot0.kD = CANCoderSensorUnitsToDegrees(0.05) * 1023.0;
        armConfig.slot0.kF = 1023.0/CANCoderSensorUnitsToDegreesPerSecond(1.301496);
        armConfig.motionCruiseVelocity = degreesPerSecondToCANCoderSensorUnits(0.2);
        armConfig.motionAcceleration = degreesPerSecondToCANCoderSensorUnits(0.4);
        armConfig.voltageCompSaturation = 10.5;
        armConfig.neutralDeadband = 0.001;
        armConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 50, 50, 1.5);
        arm.setInverted(ARM_INVERT_TYPE);
        arm.setNeutralMode(NeutralMode.Coast);
        arm.setSensorPhase(true);
        arm.enableVoltageCompensation(true);
        arm.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        arm.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
        arm.set(ControlMode.PercentOutput, 0.0);
        arm.configAllSettings(armConfig);
        arm.setSelectedSensorPosition(0.0);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.angleDegrees = armAzimuth.getPosition();
        inputs.velDegreesPerSeconds = armAzimuth.getVelocity();
        inputs.currentAmps = arm.getStatorCurrent();
        inputs.appliedVoltage = arm.getMotorOutputVoltage();
        inputs.tempCelcius = arm.getTemperature();
    }

    @Override
    public void setAngle(double angleRadians) {
        arm.set(ControlMode.Position, degreesToCANCoderSensorUnits(angleRadians));
    }

    @Override
    public void setPercent(double percent) {
        arm.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void resetArm() {
        double angle = armAzimuth.getAbsolutePosition() - ARM_AZIMUTH_DEGREE_OFFSET;
        /* puts the angle in the [0, 360] range */
        while (angle < 0.0) {
            angle = 360.0 + angle;
        }
    
        /* if the angle is in our null range, then we subtract 360 from it */
        while (angle > ARM_NULL_RANGE) {
            angle -= 360.0;
        }
        armAzimuth.setPosition(angle);
    }

    @Override
    public void enableBrakeMode(boolean enable) {
        arm.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
    }

    private double CANCoderSensorUnitsToDegrees(double sensorUnits) {
        return sensorUnits * (360.0) / 4096.0;
    }

    private double degreesToCANCoderSensorUnits(double degrees) {
        return degrees * 4096.0 / (360.0);
    }

    private double CANCoderSensorUnitsToDegreesPerSecond(double sensorUnits) {
        return sensorUnits * ((360.0 * 10.0)/4096.0);
    }

    private double degreesPerSecondToCANCoderSensorUnits(double degrees) {
        return degrees * (4096.0/(360.0 * 10.0));
    }

}

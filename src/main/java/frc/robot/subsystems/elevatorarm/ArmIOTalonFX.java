package frc.robot.subsystems.elevatorarm;
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

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commons.LoggedTunableNumber;

import static frc.robot.Constants.Arm.*;
public class ArmIOTalonFX implements ArmIO {

    TalonFX arm = new TalonFX(ARM_ID);
    CANCoder armAzimuth = new CANCoder(ARM_AZIMUTH_ID);

    double lastVelocityTarget = 0.0;

    LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", 0.015);
    LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", 0.05);
    LoggedTunableNumber kMotionCruiseVelocity = new LoggedTunableNumber("Arm/kMotionCruiseVelocity", 400.0); 
    LoggedTunableNumber kMaxAccel = new LoggedTunableNumber("Arm/kMaxAccel", 2300.0); 

    public ArmIOTalonFX() {
        /* configurations for the arm encoder */
        armAzimuth = new CANCoder(ARM_AZIMUTH_ID);
        armAzimuth.configSensorDirection(ARM_AZIMUTH_INVERTED);
        armAzimuth.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        armAzimuth.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        /* configurations for the arm motor */
        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        armConfig.remoteFilter0.remoteSensorDeviceID = armAzimuth.getDeviceID();
        armConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        armConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        armConfig.slot0.kP = CANCoderSensorUnitsToDegrees(kP.get()) * 1023.0;
        armConfig.slot0.kI = CANCoderSensorUnitsToDegrees(0) * 1023.0;
        armConfig.slot0.kD = CANCoderSensorUnitsToDegrees(kD.get()) * 1023.0;
        armConfig.slot0.kF = 0.0;
        armConfig.motionCruiseVelocity = degreesPerSecondToCANCoderSensorUnits(kMotionCruiseVelocity.get());
        armConfig.motionAcceleration = degreesPerSecondToCANCoderSensorUnits(kMaxAccel.get());
        armConfig.voltageCompSaturation = 10.5;
        armConfig.neutralDeadband = 0.001;
        armConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 100, 100, 1.5);
        arm.setInverted(ARM_INVERT_TYPE);
        arm.setNeutralMode(NeutralMode.Brake);
        arm.setSensorPhase(true);
        arm.enableVoltageCompensation(true);
        arm.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        arm.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
        arm.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10);
        arm.set(ControlMode.PercentOutput, 0.0);
        arm.configAllSettings(armConfig);
        arm.setSelectedSensorPosition(0.0);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.angleDegrees = getAngle();
        inputs.velDegreesPerSecond = CANCoderSensorUnitsToDegreesPerSecond(arm.getSelectedSensorVelocity());
        inputs.angleDegreesCC = armAzimuth.getPosition();
        inputs.velDegreesPerSecondCC = armAzimuth.getVelocity();
        inputs.currentAmps = arm.getStatorCurrent();
        inputs.appliedVoltage = arm.getMotorOutputVoltage();
        inputs.appliedPercent = arm.getMotorOutputPercent();
        inputs.tempCelcius = arm.getTemperature();
        inputs.armTargetPosition = CANCoderSensorUnitsToDegrees(arm.getActiveTrajectoryPosition());
        inputs.armTargetVelocity = CANCoderSensorUnitsToDegreesPerSecond(arm.getActiveTrajectoryVelocity());
    }

    @Override
    public void setAngle(double angleDegrees) {
        // double currentVelocityTarget = CANCoderSensorUnitsToDegreesPerSecond(arm.getActiveTrajectoryVelocity());
        // double armKA = 0.0;
        // if (currentVelocityTarget > lastVelocityTarget) {
        //     armKA = kMotionAcceleration.get() * kA.get();
        // } else if (currentVelocityTarget < lastVelocityTarget) {
        //     armKA = kMotionAcceleration.get() * -kA.get();
        // } 
        // lastVelocityTarget = currentVelocityTarget;
        // double armKG = Math.cos(getAngle()) * kG.get();
        if (!DriverStation.isEnabled()) {
            arm.set(ControlMode.PercentOutput, 0.0);
            return;
        }
        arm.set(ControlMode.MotionMagic, degreesToCANCoderSensorUnits(angleDegrees));
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

    @Override
    public void updateTunableNumbers() {
        if (kP.hasChanged(0)) {
            arm.config_kP(0, CANCoderSensorUnitsToDegrees(kP.get()) * 1023.0);
        }

        if (kD.hasChanged(0)) {
            arm.config_kD(0, CANCoderSensorUnitsToDegrees(kD.get()) * 1023.0);
        }

        if (kMotionCruiseVelocity.hasChanged(0)) {
            arm.configMotionCruiseVelocity(degreesPerSecondToCANCoderSensorUnits(kMotionCruiseVelocity.get()));
        }

        if (kMaxAccel.hasChanged(0)) {
            arm.configMotionAcceleration(degreesPerSecondToCANCoderSensorUnits(kMaxAccel.get()));
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

    private double getAngle() {
        return CANCoderSensorUnitsToDegrees(arm.getSelectedSensorPosition());
    }

}

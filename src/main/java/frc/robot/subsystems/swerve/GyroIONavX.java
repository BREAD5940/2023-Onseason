package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;

public class GyroIONavX implements GyroIO {

    AHRS gyro;

    public GyroIONavX() {
        gyro = new AHRS(Port.kMXP);
        gyro.calibrate();
        gyro.reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.positionRad = gyro.getRotation2d().getRadians(); // ?????
        inputs.positionDegRaw = -gyro.getAngle();
        inputs.velocityRadPerSec = gyro.getVelocityX(); // ?????
    }

    @Override
    public void reset() {
        gyro.reset();
        gyro.calibrate();
    }
    
}
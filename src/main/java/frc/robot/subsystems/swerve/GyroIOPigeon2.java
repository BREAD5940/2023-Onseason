package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 implements GyroIO {

    Pigeon2 pigeon;
    MedianFilter filter = new MedianFilter(10);

    public GyroIOPigeon2() {
        pigeon = new Pigeon2(30, "dabus");
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.positionDegRaw = pigeon.getYaw();
        inputs.positionRad = Units.degreesToRadians(inputs.positionDegRaw);
        inputs.velocityRadPerSec = 0.0;
        inputs.pitchDeg = pigeon.getPitch();
        inputs.rollDeg = pigeon.getRoll() - 3.515625;
        inputs.pitchRad = Units.degreesToRadians(pigeon.getPitch());
        inputs.rollRad = Units.degreesToRadians(inputs.rollDeg);
        pigeon.getRawGyro(inputs.xyz_dps);
        inputs.changeInPitch = filter.calculate(inputs.xyz_dps[1]);
    }

    @Override
    public void reset() {
        pigeon.setYaw(0.0);
    }
    
}

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class BeamBreakIOTalonFX implements BeamBreakIO {
	TalonFX talonFX;
	public BeamBreakIOTalonFX(TalonFX talonFX) {
		this.talonFX = talonFX;
	}

	@Override
	public boolean isDetecting() {
		return talonFX.getSensorCollection().isFwdLimitSwitchClosed() == 1;
	}
}

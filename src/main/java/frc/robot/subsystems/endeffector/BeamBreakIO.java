package frc.robot.subsystems.endeffector;

/** Used to interface with a beam break */
public interface BeamBreakIO {
	/**
	 * Used to get the current state of the beam break
	 * @return
	 * returns true if the beam is broken(an obstruction in the way of the beam)
	 */
	public default boolean isDetecting() {return false;}
}

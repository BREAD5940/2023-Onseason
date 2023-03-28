package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.Logger;

import frc.robot.commons.LoggedTunableNumber;

import com.ctre.phoenix.ErrorCode;

/** a class that is used to mannage a FloorIntakeIO */
public class EndEffector {

    /* IO classes and inputs */
    private EndEffectorIO endEffectorIO;
    private EndEffectorIOInputsAutoLogged endEffectorInputs = new EndEffectorIOInputsAutoLogged();
    
    /* Tunable numbers */
    LoggedTunableNumber intakeConePercent = new LoggedTunableNumber("EndEffector/IntakeConePercent", 1.0);
    LoggedTunableNumber intakeCubePercent = new LoggedTunableNumber("EndEffector/IntakeCubePercent", 0.9);
    LoggedTunableNumber holdingPercent = new LoggedTunableNumber("EndEffector/HoldingPercent", 0.1);
    LoggedTunableNumber spitCubePercent = new LoggedTunableNumber("EndEffector/SpitCubePercent", -1.0);

    // For fault detection
    private int ErrCount = 0;
    private int errCheckNum = 1;


    /** Instantiate the IO instance in the constructor */
    public EndEffector(EndEffectorIO endEffectorIO) {
        this.endEffectorIO = endEffectorIO;
    }

    /** Method to be called periodically */
    public void onLoop() {
        endEffectorIO.updateInputs(endEffectorInputs);
        Logger.getInstance().processInputs("EndEffector", endEffectorInputs);
        endEffectorIO.updateFilter();

        if(endEffectorInputs.lastError != ErrorCode.OK.toString()) {
            ErrCount++;
        }
        endEffectorIO.clearFault();
        errCheckNum++;
    }

    /** For intaking cones */
    public void intakeCone() {
        endEffectorIO.setPercent(intakeConePercent.get());
        endEffectorIO.setCurrentLimit( 180, 200.0);
    }

    /** For intaking cubes */
    public void intakeCube() {
        endEffectorIO.setPercent(intakeCubePercent.get());
        endEffectorIO.setCurrentLimit(20, 30.0);
    }

    /** For holding a game piece */
    public void holdGamePiece() {
        endEffectorIO.setPercent(holdingPercent.get());
        endEffectorIO.setCurrentLimit(10, 15.0);
    }

    /* For spitting a cube */
    public void spitCube() {
        endEffectorIO.setPercent(spitCubePercent.get());
        endEffectorIO.setCurrentLimit(80, 100.0);
    }

    /** For idling */
    public void idling() {
        endEffectorIO.setPercent(0.0);
        endEffectorIO.setCurrentLimit(20, 30.0);
    }

    /** For holding a cone while moving the elevator */
    public void holdConeElevatorMoving() {
         endEffectorIO.setPercent(holdingPercent.get());
         endEffectorIO.setCurrentLimit(30, 40.0);
    }

    /** Enables/disables brake mode */
    public void enableBrakeMode(boolean enable) {
        endEffectorIO.enableBrakeMode(enable);
    }

    /** Returns the stator current of the end effector */
    public double getStatorCurrent() {
        return endEffectorInputs.avgStatorCurrentAmps;
    }

    /** Returns the Error concentration for the end effector motor */
    public double getEndEffectorErrorConc(){
        return(ErrCount/errCheckNum);
    }

    /** Resets error counters */
    public void resetError(){
        ErrCount = 0;
        errCheckNum = 1;
    }

	/**
	 * used to check if the end effector is holding a cone
	 * @return
	 * returns true if the end effector is holding a cone
	 */
	public boolean isHoldingCone() {
		return endEffectorIO.isHoldingCone();
	}
}

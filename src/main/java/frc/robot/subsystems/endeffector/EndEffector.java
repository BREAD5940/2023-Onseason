package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.Logger;

import frc.robot.commons.TunableNumber;

public class EndEffector {

    /* IO classes and inputs */
    private EndEffectorIO endEffectorIO;
    private EndEffectorIOInputsAutoLogged endEffectorInputs = new EndEffectorIOInputsAutoLogged();
    
    /* Tunable numbers */
    TunableNumber intakeConePercent = new TunableNumber("EndEffector/IntakeConePercent", 0.9);
    TunableNumber intakeCubePercent = new TunableNumber("EndEffector/IntakeCubePercent", 0.9);
    TunableNumber holdingPercent = new TunableNumber("EndEffector/HoldingPercent", 0.9);
    TunableNumber spitCubePercent = new TunableNumber("EndEffector/SpitCubePercent", -0.3);


    /* Instantiate the IO instance in the constructor */
    public EndEffector(EndEffectorIO endEffectorIO) {
        this.endEffectorIO = endEffectorIO;
    }

    /* Method to be called periodically */
    public void onLoop() {
        endEffectorIO.updateInputs(endEffectorInputs);
        Logger.getInstance().processInputs("EndEffector", endEffectorInputs);
    }

    /* For intaking cones */
    public void intakeCone() {
        endEffectorIO.setPercent(intakeConePercent.get());
    }

    /* For intaking cubes */
    public void intakeCube() {
        endEffectorIO.setPercent(intakeCubePercent.get());
    }

    /* For holding a game piece */
    public void holdGamePiece() {
        endEffectorIO.setPercent(holdingPercent.get());
    }

    /* For spitting a cube */
    public void spitCube() {
        endEffectorIO.setPercent(spitCubePercent.get());
    }

    /* For idling */
    public void idling() {
        endEffectorIO.setPercent(0.0);
    }

    /* Enables/disables brake mode */
    public void enableBrakeMode(boolean enable) {
        endEffectorIO.enableBrakeMode(enable);
    }
}

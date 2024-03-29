package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.Logger;

import frc.robot.commons.LoggedTunableNumber;

public class EndEffector {

    /* IO classes and inputs */
    private EndEffectorIO endEffectorIO;
    private EndEffectorIOInputsAutoLogged endEffectorInputs = new EndEffectorIOInputsAutoLogged();
    
    /* Tunable numbers */
    LoggedTunableNumber intakeConePercent = new LoggedTunableNumber("EndEffector/IntakeConePercent", 1.0);
    LoggedTunableNumber intakeCubePercent = new LoggedTunableNumber("EndEffector/IntakeCubePercent", 0.5);

    LoggedTunableNumber holdingPercent = new LoggedTunableNumber(
        "EndEffector/HoldingPercent", 0.1);
    LoggedTunableNumber spitCubePercent = new LoggedTunableNumber("EndEffector/SpitCubePercent", -0.35);
    LoggedTunableNumber spitLowPercent = new LoggedTunableNumber("EndEffector/SpitLowPercent", -0.15);


    /* Instantiate the IO instance in the constructor */
    public EndEffector(EndEffectorIO endEffectorIO) {
        this.endEffectorIO = endEffectorIO;
    }

    /* Method to be called periodically */
    public void onLoop() {
        endEffectorIO.updateInputs(endEffectorInputs);
        Logger.getInstance().processInputs("EndEffector", endEffectorInputs);
        endEffectorIO.updateFilter();
    }

    /* For intaking cones */
    public void intakeCone() {
        endEffectorIO.setPercent(intakeConePercent.get());
        endEffectorIO.setCurrentLimit( 180, 200.0);
    }

    /* For intaking cubes */
    public void intakeCube() {
        endEffectorIO.setPercent(intakeCubePercent.get());
        endEffectorIO.setCurrentLimit(20, 30.0);
    }

    /* For holding a cube */
    public void holdCube() {
        endEffectorIO.setPercent(holdingPercent.get());
        endEffectorIO.setCurrentLimit(10, 15.0);
    }

    /* For holding a cone */
    public void holdCone() {
        endEffectorIO.setPercent(holdingPercent.get());
        endEffectorIO.setCurrentLimit(20, 30.0);
    }

    /* For spitting a cube */
    public void spitCube() {
        endEffectorIO.setPercent(spitCubePercent.get());
        endEffectorIO.setCurrentLimit(80, 100.0);
    }

    /* For spitting a piece low */
    public void spitLow() {
        endEffectorIO.setPercent(spitLowPercent.get());
        endEffectorIO.setCurrentLimit(80, 100.0);
    }

    /* For idling */
    public void idling() {
        endEffectorIO.setPercent(0.0);
        endEffectorIO.setCurrentLimit(0.0, 0.0);
    }

    /* For holding a cone while moving the elevator */
    public void holdConeElevatorMoving() {
         endEffectorIO.setPercent(holdingPercent.get());
         endEffectorIO.setCurrentLimit(30, 40.0);
    }

    /* For throwing cubes */
    public void throwCube() {
        endEffectorIO.setPercent(-1.0);
        endEffectorIO.setCurrentLimit(180, 200.0);
    }

    /* Enables/disables brake mode */
    public void enableBrakeMode(boolean enable) {
        endEffectorIO.enableBrakeMode(enable);
    }

    /* Returns the stator current of the end effector */
    public double getStatorCurrent() {
        return endEffectorInputs.avgStatorCurrentAmps;
    }

    /* Returns the rpm of the motor */
    public double getMotorRPM() {
        return endEffectorInputs.motorSpeed;
    }

    /* Returns whether the end effector's beam break is triggered */
    public boolean getBeamBreakTriggered() {
        return endEffectorInputs.beamBreakTriggered;
    }
}

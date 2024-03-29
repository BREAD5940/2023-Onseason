package frc.robot.subsystems.floorintake;

import org.littletonrobotics.junction.Logger;

import frc.robot.commons.BreadUtil;

import static frc.robot.Constants.FloorIntake.*;

public class FloorIntake {

    /* IO and inputs classes */
    public FloorIntakeIO floorIntakeIO;
    public FloorIntakeIOInputsAutoLogged floorIntakeInputs = new FloorIntakeIOInputsAutoLogged();

    /* Variables to keep track of system state */
    private double mStateStartTime = 0.0;
    private FloorIntakeStates systemState = FloorIntakeStates.IDLE;
    private boolean requestIdle = true;
    private boolean requestClosedLoop = false;
    private double[] closedLoopSetpoint = new double[] { 0.0, 0.0 };

    public enum FloorIntakeStates {
        IDLE,
        CLOSED_LOOP

    }

    /* Instantiate IO class in the constructor */
    public FloorIntake(FloorIntakeIO floorIntakeIO) {
        this.floorIntakeIO = floorIntakeIO;
    }

    /* This onLoop() method is to be called periodically */
    public void onLoop() {
        floorIntakeIO.updateInputs(floorIntakeInputs);
        floorIntakeIO.updateTunableNumbers();
        Logger.getInstance().processInputs("FloorIntake", floorIntakeInputs);
        Logger.getInstance().recordOutput("FloorIntakeSetpoint", closedLoopSetpoint[1]);
        Logger.getInstance().recordOutput("FloorIntakeState", systemState.toString());

        FloorIntakeStates nextSystemState = systemState;

        if (systemState == FloorIntakeStates.IDLE) {
            // Outputs
            floorIntakeIO.setRollerPercent(0.0);
            floorIntakeIO.setDeployAngle(INTAKE_IDLE_POSITION);
            floorIntakeIO.setCurrentLimit(50.0, 60.0, 1.5);

            // Transitions
            if (requestClosedLoop) {
                nextSystemState = FloorIntakeStates.CLOSED_LOOP;
            }
        } else if (systemState == FloorIntakeStates.CLOSED_LOOP) {
            // Outputs
            floorIntakeIO.setRollerPercent(closedLoopSetpoint[0]);
            floorIntakeIO.setDeployAngle(closedLoopSetpoint[1]);
            floorIntakeIO.setCurrentLimit(50.0, 60.0, 1.5);

            // Transitions
            if (requestIdle) {
                nextSystemState = FloorIntakeStates.IDLE;
            }
        }

        if (nextSystemState != systemState) {
            mStateStartTime = BreadUtil.getFPGATimeSeconds();
            systemState = nextSystemState;
        }
    }

    /* Requests the intake to go into its idling mode */
    public void requestIdle() {
        requestIdle = true;
        requestClosedLoop = false;
    }

    /* Requests the intake to go into its closed loop mode */
    public void requestClosedLoop(double rollerPercent, double deployAngle) {
        closedLoopSetpoint = new double[] { rollerPercent, deployAngle };
        requestClosedLoop = true;
        requestIdle = false;
    }

    /** Returns the system state of the intake */
    public FloorIntakeStates getSystemState() {
        return systemState;
    }

    /** Returns the current of the roller */
    public double getRollerCurrent() {
        return floorIntakeInputs.rollerCurrentAmps;
    }

    /** Returns the angle of the floor intake */
    public double getAngle() {
        return floorIntakeInputs.angleDegrees;
    }

    /** Returns the speed of the floor intake's motor in rpm */
    public double getRollerRPM() {
        return floorIntakeInputs.rollerMotorSpeedRPM;
    }

    /** Enables coast mode on the intake */
    public void requestBrakeMode(boolean enable) {
        floorIntakeIO.enableDeployBrakeMode(enable);
    }

    /** Zeros sensors */
    public void zeroSensors() {
        floorIntakeIO.resetAngle();
    }

}

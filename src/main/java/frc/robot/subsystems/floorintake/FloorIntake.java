package frc.robot.subsystems.floorintake;

import org.littletonrobotics.junction.Logger;

import frc.robot.commons.BreadUtil;

public class FloorIntake {

    /* IO and inputs classes */
    public FloorIntakeIO floorIntakeIO;
    public FloorIntakeIOInputsAutoLogged floorIntakeInputs = new FloorIntakeIOInputsAutoLogged();

    /* Variables to keep track of system state */
    private double mStateStartTime = 0.0;
    private FloorIntakeStates systemState = FloorIntakeStates.PRE_HOME;
    private boolean requestHome = true;
    private boolean requestIdle = false;
    private boolean requestClosedLoop = false;
    private double[] closedLoopSetpoint = new double[] {0.0, 0.0};

    public enum FloorIntakeStates {
        PRE_HOME,
        HOMING,
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
        Logger.getInstance().processInputs("FloorIntake", floorIntakeInputs);
        Logger.getInstance().recordOutput("FloorIntakeSetpoint", closedLoopSetpoint[1]);

        FloorIntakeStates nextSystemState = systemState;

        if (systemState == FloorIntakeStates.PRE_HOME) {
            // Outputs
            floorIntakeIO.setRollerPercent(0.0);
            floorIntakeIO.setDeployPercent(0.0);

            // Transitions
            if (requestHome) {
                nextSystemState = FloorIntakeStates.HOMING;
            }
        } else if (systemState == FloorIntakeStates.HOMING) {
            // Outputs
            floorIntakeIO.setRollerPercent(0.0);
            floorIntakeIO.setDeployPercent(-0.3);

            // Transitions
            if (BreadUtil.getFPGATimeSeconds() - mStateStartTime > 0.25 && Math.abs(floorIntakeInputs.deployVelDegreesPerSecond) < 5.0) {
                floorIntakeIO.resetDeployAngle(0.0);
                nextSystemState = FloorIntakeStates.IDLE;
                requestHome = false;
            }
        } else if (systemState == FloorIntakeStates.IDLE) {
            // Outputs
            floorIntakeIO.setRollerPercent(0.0);
            floorIntakeIO.setDeployPercent(0.0);

            // Transitions
            if (requestHome) {
                nextSystemState = FloorIntakeStates.HOMING;
            } else if (requestClosedLoop) {
                nextSystemState = FloorIntakeStates.CLOSED_LOOP;
            }
        } else if (systemState == FloorIntakeStates.CLOSED_LOOP) {
            // Outputs
            floorIntakeIO.setRollerPercent(closedLoopSetpoint[0]);
            floorIntakeIO.setDeployAngle(closedLoopSetpoint[1]);

            // Transitions
            if (requestHome) {
                nextSystemState = FloorIntakeStates.HOMING;
            } else if (requestIdle) {
                nextSystemState = FloorIntakeStates.IDLE;
            }
        }

        if (nextSystemState != systemState) {
            mStateStartTime = BreadUtil.getFPGATimeSeconds();
            systemState = nextSystemState;
        }
    }

    /* Requests the intake to home */
    public void requestHome() {
        requestHome = true;
    }

    /* Requests the intake to go into its idling mode */
    public void requestIdle() {
        requestIdle = true;
        requestClosedLoop = false;
    }

    /* Requests the intake to go into its closed loop mode */
    public void requestClosedLoop(double rollerPercent, double deployAngle) {
        closedLoopSetpoint = new double[] {rollerPercent, deployAngle};
        requestClosedLoop = true;
        requestIdle = false;
    }

    /** Returns the system state of the intake */
    public FloorIntakeStates getSystemState() {
        return systemState;
    }
    
}

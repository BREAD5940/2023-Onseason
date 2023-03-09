package frc.robot.subsystems.floorintake;

import org.littletonrobotics.junction.Logger;

import frc.robot.commons.BreadUtil;
import frc.robot.commons.LoggedTunableNumber;

import static frc.robot.Constants.FloorIntake.*;

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
            floorIntakeIO.setCurrentLimit(5.0, 10.0, 0.0);

            // Transitions
            if (requestHome) {
                nextSystemState = FloorIntakeStates.HOMING;
            }
        } else if (systemState == FloorIntakeStates.HOMING) {
            // Outputs
            floorIntakeIO.setRollerPercent(0.0);
            floorIntakeIO.setDeployVelocity(-50.0);
            floorIntakeIO.setCurrentLimit(20.0, 25.0, 0.0);

            // Transitions
            if (BreadUtil.getFPGATimeSeconds() - mStateStartTime > 0.25 && Math.abs(floorIntakeInputs.deployVelDegreesPerSecond) < 10.0) {
                floorIntakeIO.resetDeployAngle(FLOOR_INTAKE_ZERO);
                nextSystemState = FloorIntakeStates.IDLE;
                requestHome = false;
            }
        } else if (systemState == FloorIntakeStates.IDLE) {
            // Outputs
            floorIntakeIO.setRollerPercent(0.0);
            floorIntakeIO.setDeployPercent(0.0);
            floorIntakeIO.setCurrentLimit(5.0, 10.0, 0.0);

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
            floorIntakeIO.setCurrentLimit(50.0, 60.0, 1.5);

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
        requestIdle = false;
        requestClosedLoop = false;
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

    /** Returns the current of the roller */
    public double getRollerCurrent() {
        return floorIntakeInputs.rollerCurrentAmps;
    }

    /** Returns the angle of the floor intake */
    public double getAngle() {
        return floorIntakeInputs.deployAngleDegrees;
    }

    /** Enables coast mode on the intake */
    public void requestBrakeMode(boolean enable) {
        floorIntakeIO.enableDeployBrakeMode(enable);
    }
    
}

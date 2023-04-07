package frc.robot.subsystems.floorintake;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commons.BreadUtil;
import frc.robot.commons.LoggedTunableNumber;

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

    // For fault detection
    private int deployErrCount = 0;
    private int rollerErrCount = 0;
    private int errCheckNum = 1;

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

        if(floorIntakeInputs.lastDeployError != ErrorCode.OK.toString()){
            deployErrCount++;
        }
        if(floorIntakeInputs.lastRollerError != ErrorCode.OK.toString()){
            rollerErrCount++;
        }
        errCheckNum++;

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

    /** Returns the Error concentration for the intake roller motor */
    public double getRollerErrorConc() {
        return (rollerErrCount / errCheckNum);
    }

    /** Returns the Error concentration for the intake deploy motor */
    public double getDeployErrorConc() {
        return (deployErrCount / errCheckNum);
    }

    /** Resets error counters */
    public void resetError() {
        deployErrCount = 0;
        rollerErrCount = 0;
        errCheckNum = 1;
    }

	/** runs code that is needed for test mode. onLoop should not run while running this */
	public void runTestMode(boolean on) {
		if (on) {
			floorIntakeIO.updateInputs(floorIntakeInputs);
			floorIntakeIO.setRollerPercent(0.1);
			floorIntakeIO.setDeployPercent(-0.03);
			SmartDashboard.putNumber("rollerCurrentAmps", floorIntakeInputs.rollerCurrentAmps);
			SmartDashboard.putNumber("deployCurrentAmps", floorIntakeInputs.deployCurrentAmps);
		} else {
			floorIntakeIO.updateInputs(floorIntakeInputs);
			floorIntakeIO.setRollerPercent(0.0);
			floorIntakeIO.setDeployPercent(0.0);
		}
	}
}

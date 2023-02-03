package frc.robot.subsystems.superstructure;

import static frc.robot.Constants.Arm.*;
import static frc.robot.Constants.Elevator.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import frc.robot.commons.BreadUtil;

public class ElevatorArmLowLevel {

    /** State variables */
    private double mStateStartTime = 0.0;
    private ElevatorArmSystemStates systemState = ElevatorArmSystemStates.STARTING_CONFIG;
    private ElevatorArmState desiredState = new ElevatorArmState(0.0, 0.0);
    private boolean requestHome = false;
    private boolean requestSetpointFollower = false;
    private boolean requestIdle = false;

    /** Instantiate the IO classes */
    public ArmIO armIO;
    public ElevatorIO elevatorIO;

    public ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
    public ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    

    /** Define variables in constructor */
    public ElevatorArmLowLevel(ArmIO armIO, ElevatorIO elevatorIO) {
        this.armIO = armIO;
        this.elevatorIO = elevatorIO;
    }

    /** System states */
    public enum ElevatorArmSystemStates {
        STARTING_CONFIG, 
        NEUTRALIZING_ARM, 
        HOMING,
        IDLE, 
        FOLLOWING_SETPOINT
    }

    /** Method to be called periodically */
    public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);
        armIO.updateInputs(armInputs);
        Logger.getInstance().processInputs("Elevator", elevatorInputs);
        Logger.getInstance().processInputs("ArmInputs", armInputs);

        ElevatorArmSystemStates nextSystemState = systemState;

        if (systemState == ElevatorArmSystemStates.STARTING_CONFIG) {
            elevatorIO.setPercent(0.0);
            armIO.setPercent(0.0);

            if (requestHome) {
                nextSystemState = ElevatorArmSystemStates.NEUTRALIZING_ARM;
            }

        } else if (systemState == ElevatorArmSystemStates.NEUTRALIZING_ARM) {
            elevatorIO.setPercent(0.0);
            armIO.setAngle(ARM_NEUTRAL_ANGLE);

            if (atArmSetpoint(ARM_NEUTRAL_ANGLE)) {
                // nextSystemState = ElevatorArmSystemStates.HOMING;
            }
        } else if (systemState == ElevatorArmSystemStates.HOMING) {
            elevatorIO.setPercent(-0.3);
            armIO.setAngle(ARM_NEUTRAL_ANGLE);

            if (BreadUtil.getFPGATimeSeconds() - mStateStartTime < 0.25 && elevatorInputs.velMetersPerSecond < 0.1) {
                nextSystemState = ElevatorArmSystemStates.IDLE;
                requestHome = false;
            }
        } else if (systemState == ElevatorArmSystemStates.IDLE) {
            elevatorIO.setPercent(0.0);
            armIO.setPercent(0.0);

            if (requestHome) {
                nextSystemState = ElevatorArmSystemStates.NEUTRALIZING_ARM;
            } else if (requestSetpointFollower) {
                nextSystemState = ElevatorArmSystemStates.FOLLOWING_SETPOINT;
            }
        } else if (systemState == ElevatorArmSystemStates.FOLLOWING_SETPOINT) {
            elevatorIO.setHeight(desiredState.elevatorHeight);
            armIO.setAngle(desiredState.armAngle);

            if (requestHome) {
                nextSystemState = ElevatorArmSystemStates.NEUTRALIZING_ARM;
            } else if (requestIdle) {
                nextSystemState = ElevatorArmSystemStates.IDLE;
            }
        }

        if (nextSystemState != systemState) {
            systemState = nextSystemState;
            mStateStartTime = BreadUtil.getFPGATimeSeconds();
        }

    }    

    /** Requests a desired state for the elevator + arm and clamps it if it's not valid */
    public void requestDesiredState(double elevatorHeight, double armAngle) {
        requestSetpointFollower = true;
        requestIdle = false;
        requestSetpointFollower = true;
        elevatorHeight = MathUtil.clamp(elevatorHeight, armInputs.angleDegrees > ARM_MAX_LIMITED_ELEVATOR_ROM ? ELEVATOR_MIN_LIMITED_ARM_ROM : ELEVATOR_MIN, ELEVATOR_MAX);
        armAngle = MathUtil.clamp(armAngle, 0.0, elevatorInputs.posMeters < ELEVATOR_MIN_LIMITED_ARM_ROM ? ARM_MAX_LIMITED_ELEVATOR_ROM : ARM_MAX);
        desiredState = new ElevatorArmState(elevatorHeight, armAngle);
    }

    /** Requests the elevator + arm to go into idle mode */
    public void requestIdle() {
        requestIdle = true;
        requestSetpointFollower = false;
    }

    /** Requests the elevator + arm to home */
    public void requestHome() {
        requestHome = true;
    }

    /** Returns whether or not the arm is at a certain position */
    public boolean atArmSetpoint(double setpoint) {
        return BreadUtil.atReference(armInputs.angleDegrees, setpoint, ARM_SETPOINT_TOLERANCE, true);
    }

    /** Returns whether or not the elevator is at a certain position */
    public boolean atElevatorSetpoint(double setpoint) {
        return BreadUtil.atReference(elevatorInputs.posMeters, setpoint, ELEVATOR_SETPOINT_TOLERANCE, true);
    }

    /** Returns the desired state of the elevator */
    public ElevatorArmState getDesiredState(double elevatorHeight, double armAngle) {
        return desiredState;
    }

    /** Returns the current state of the elevator */
    public ElevatorArmState getState() {
        return new ElevatorArmState(elevatorInputs.posMeters, armInputs.angleDegrees);
    }

    /** Record for keeping track of elevator state */
    public record ElevatorArmState(double elevatorHeight, double armAngle) { }

}

package frc.robot.subsystems.elevatorarm;

import static frc.robot.Constants.Arm.*;
import static frc.robot.Constants.Elevator.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.commons.LoggedTunableNumber;

public class ElevatorArmLowLevel {

    /** State variables */
    private double mStateStartTime = 0.0;
    private ElevatorArmSystemStates systemState = ElevatorArmSystemStates.STARTING_CONFIG;
    private double[] desiredState = new double[] {0.0, 0.0};
    private boolean requestHome = false;
    private boolean requestSetpointFollower = false;
    private boolean requestIdle = false;
    private boolean moveElevatorSlowly = false;
    private double heightSetpoint = 0.0;
    private double angleSetpoint = 0.0;

    /** Instantiate the IO classes */
    public ArmIO armIO;
    public ElevatorIO elevatorIO;

    public ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
    public ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    LoggedTunableNumber armOutput = new LoggedTunableNumber("Arm/ArmOutput", 0.0);
    LoggedTunableNumber elevatorOutput = new LoggedTunableNumber("Elevator/ElevatorOutput", 0.0);
    

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
    public void onLoop() {
        double elevatorHeight = MathUtil.clamp(heightSetpoint, armInputs.angleDegrees > ARM_MAX_LIMITED_ELEVATOR_ROM ? ELEVATOR_MIN_LIMITED_ARM_ROM : ELEVATOR_MIN, ELEVATOR_MAX);
        double armAngle = MathUtil.clamp(angleSetpoint, ARM_MIN, elevatorInputs.posMeters < ELEVATOR_MIN_LIMITED_ARM_ROM ? ARM_MAX_LIMITED_ELEVATOR_ROM : ARM_MAX);
        desiredState = new double[] {elevatorHeight, armAngle};

        elevatorIO.updateInputs(elevatorInputs);
        elevatorIO.updateTunableNumbers();
        armIO.updateTunableNumbers();
        armIO.updateInputs(armInputs);

        Logger.getInstance().processInputs("Elevator", elevatorInputs);
        Logger.getInstance().processInputs("Arm", armInputs);
        Logger.getInstance().recordOutput("ElevatorArmLowLevelState", systemState.toString());
        Logger.getInstance().recordOutput("ElevatorSetpoint", desiredState[0]);
        Logger.getInstance().recordOutput("ArmSetpoint", desiredState[1]);

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
                nextSystemState = ElevatorArmSystemStates.HOMING;
            }
        } else if (systemState == ElevatorArmSystemStates.HOMING) {
            elevatorIO.setPercent(-0.3);
            armIO.setAngle(ARM_NEUTRAL_ANGLE);

            if (BreadUtil.getFPGATimeSeconds() - mStateStartTime > 0.25 && Math.abs(elevatorInputs.velMetersPerSecond) < 0.1) {
                elevatorIO.resetHeight(ELEVATOR_HOMING_POSITION);
                nextSystemState = ElevatorArmSystemStates.IDLE;
                requestHome = false;
            }
        } else if (systemState == ElevatorArmSystemStates.IDLE) {
            RobotContainer.armIO.setAngle(90.0);   
            RobotContainer.elevatorIO.setPercent(0.0);

            if (requestHome) {
                nextSystemState = ElevatorArmSystemStates.NEUTRALIZING_ARM;
            } else if (requestSetpointFollower) {
                nextSystemState = ElevatorArmSystemStates.FOLLOWING_SETPOINT;
            }
        } else if (systemState == ElevatorArmSystemStates.FOLLOWING_SETPOINT) {
            elevatorIO.setHeight(desiredState[0], moveElevatorSlowly);
            armIO.setAngle(desiredState[1]);

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
    public void requestDesiredState(double elevatorHeight, double armAngle, boolean goSlow) {
        requestSetpointFollower = true;
        requestIdle = false;
        heightSetpoint = elevatorHeight;
        angleSetpoint = armAngle;
        moveElevatorSlowly = goSlow;
    }

    /** Requests the elevator + arm to go into idle mode */
    public void requestIdle() {
        requestIdle = true;
        requestSetpointFollower = false;
    }

    /** Requests the elevator + arm to home */
    public void requestHome() {
        requestHome = true;
        requestSetpointFollower = false;
        requestIdle = false;
    }

    /** Zeros sensors */
    public void zeroSensors() {
        armIO.resetArm();
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
    public double[] getDesiredState() {
        return desiredState;
    }

    /** Returns the current state of the elevator */
    public double[] getState() {
        return new double[] {elevatorInputs.posMeters, armInputs.angleDegrees};
    }

    /** Returns the height of the elevator */

    /** Returns the system state of the elevator and arm */
    public ElevatorArmSystemStates getSystemState() {
        return systemState;
    }

}

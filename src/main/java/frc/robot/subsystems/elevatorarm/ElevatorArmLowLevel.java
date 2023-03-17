package frc.robot.subsystems.elevatorarm;

import static frc.robot.Constants.Arm.*;
import static frc.robot.Constants.Elevator.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.ErrorCode;


import edu.wpi.first.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;

public class ElevatorArmLowLevel {

    /** State variables */
    private double mStateStartTime = 0.0;
    private ElevatorArmSystemStates systemState = ElevatorArmSystemStates.STARTING_CONFIG;
    private double[] desiredState = new double[] {0.0, 0.0};
    private boolean requestHome = false;
    private boolean requestSetpointFollower = false;
    private boolean requestIdle = false;
    private double heightSetpoint = 0.0;
    private double angleSetpoint = 0.0;

    /** Instantiate the IO classes */
    public ArmIO armIO;
    public ElevatorIO elevatorIO;

    public ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();
    public ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

    // For fault detection
    private int followerErrCount = 0;
    private int leaderErrCount = 0;
    private int armErrCount = 0;
    private int armAzimuthErrCount = 0;
    private int errCheckNum = 1;
    

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
        armIO.updateInputs(armInputs);

        /* Fault Handling stuffs */
        if(elevatorInputs.lastFollowerError != ErrorCode.OK){
            followerErrCount++;
        }
        if(elevatorInputs.lastLeaderError != ErrorCode.OK){
            leaderErrCount++;
        }
        if(armInputs.lastArmError != ErrorCode.OK){
            armAzimuthErrCount++;
        }
        if(armInputs.lastArmError != ErrorCode.OK){
            armErrCount++;
        }
        errCheckNum++;
        
        elevatorIO.clearFault();
        armIO.clearFault();

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
            elevatorIO.setPercent(-0.2);
            armIO.setAngle(ARM_NEUTRAL_ANGLE);

            if (BreadUtil.getFPGATimeSeconds() - mStateStartTime > 0.25 && Math.abs(elevatorInputs.velMetersPerSecond) < 0.1) {
                elevatorIO.resetHeight(0.0);
                nextSystemState = ElevatorArmSystemStates.IDLE;
                requestHome = false;
            }
        } else if (systemState == ElevatorArmSystemStates.IDLE) {
            RobotContainer.armIO.setAngle(90.0);

            if (RobotContainer.driver.getRightTriggerAxis() > 0.1) {
                RobotContainer.elevatorIO.setPercent(RobotContainer.driver.getRightTriggerAxis() * 0.3);
            } else if (RobotContainer.driver.getLeftTriggerAxis() > 0.1) {
                RobotContainer.elevatorIO.setPercent(-RobotContainer.driver.getLeftTriggerAxis() * 0.3);
            } else {    
                RobotContainer.elevatorIO.setPercent(0.0);
            }

            if (requestHome) {
                nextSystemState = ElevatorArmSystemStates.NEUTRALIZING_ARM;
            } else if (requestSetpointFollower) {
                nextSystemState = ElevatorArmSystemStates.FOLLOWING_SETPOINT;
            }
        } else if (systemState == ElevatorArmSystemStates.FOLLOWING_SETPOINT) {
            elevatorIO.setHeight(desiredState[0]);
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
    public void requestDesiredState(double elevatorHeight, double armAngle) {
        requestSetpointFollower = true;
        requestIdle = false;
        requestSetpointFollower = true;
        heightSetpoint = elevatorHeight;
        angleSetpoint = armAngle;
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

    /** Returns the system state of the elevator and arm */
    public ElevatorArmSystemStates getSystemState() {
        return systemState;
    }

    /* Returns the error concentration for the arm motor */
    public double getArmErrorConc(){
        return(armErrCount/errCheckNum);
    }

    /* Returns the error concentration for the arm encoder */
    public double getArmAzimuthErrorConc(){
        return(armAzimuthErrCount/errCheckNum);
    }

    /* Returns the error concentration for the leading elevator motor */
    public double getLeaderErrorConc(){
        return(leaderErrCount/errCheckNum);
    }

    /* Returns the Error concentration for the following elevator motor */
    public double getFollowerErrorConc(){
        return(followerErrCount/errCheckNum);
    }

    /* Resets error counters */
    public void resetError(){
        armErrCount = 0;
        armAzimuthErrCount = 0;
        followerErrCount = 0;
        leaderErrCount = 0;
        errCheckNum = 1;
    }
 
}

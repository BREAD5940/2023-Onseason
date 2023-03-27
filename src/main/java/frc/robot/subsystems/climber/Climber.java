package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.BreadUtil;

import static frc.robot.Constants.Climber.*;

import com.ctre.phoenix.ErrorCode;

/** a class that is used to mannage a ClimberIO */
public class Climber extends SubsystemBase {

    /* IO and inputs classes */
    public ClimberIO climberIO;
    public ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

    /* Variables to keep track of system state */
    private double mStateStartTime = 0.0;
    private boolean requestDeploy = false;
    private boolean requestRun = false;
    private boolean requestIdle = false;
    private double openLoopRunPercent = 0.0;
    private ClimberStates systemState = ClimberStates.RETRACTED;

    // For fault detection
    private int climberErrCount = 0;
    private int errCheckNum = 1;

    public enum ClimberStates {
        RETRACTED,
        DEPLOYING,
        IDLE, 
        RUNNING
    }

    /** Instantiate IO class in the constructor */
    public Climber(ClimberIO climberIO) {
        this.climberIO = climberIO;
    }

    /** Periodic method */
    @Override
    public void periodic() {
        long start = Logger.getInstance().getRealTimestamp();

        climberIO.updateInputs(climberInputs);
        Logger.getInstance().processInputs("Climber", climberInputs);
        Logger.getInstance().recordOutput("ClimberState", systemState.toString());

        /* Fault Handling stuffs */
        if(climberInputs.lastClimberError != ErrorCode.OK.toString()){
            climberErrCount++;
        }
        climberIO.clearFault();
        errCheckNum++;

        ClimberStates nextSystemState = systemState;

        if (systemState == ClimberStates.RETRACTED) {
            climberIO.enableBrakeMode(true);
            climberIO.setCurrentLimits(1, 1, 1.5);
            climberIO.setPercent(-0.1);

            if (requestDeploy) {
                nextSystemState = ClimberStates.DEPLOYING;
            }
        } else if (systemState == ClimberStates.DEPLOYING) {
            climberIO.enableBrakeMode(true);
            climberIO.setHeight(CLIMBER_DEPLOY_HEIGHT);
            climberIO.setCurrentLimits(60, 70, 1.5);

            if (atSetpoint(CLIMBER_DEPLOY_HEIGHT)) {
                nextSystemState = ClimberStates.IDLE;
            }
        } else if (systemState == ClimberStates.IDLE) {
            climberIO.enableBrakeMode(true);
            climberIO.setPercent(0.0);
            climberIO.setCurrentLimits(120, 130, 1.5);

            if (requestRun) {
                nextSystemState = ClimberStates.RUNNING;
            }
        } else if (systemState == ClimberStates.RUNNING) {
            climberIO.enableBrakeMode(true);
            climberIO.setPercent(openLoopRunPercent);
            climberIO.setCurrentLimits(120, 130, 1.5);

            if (requestIdle) {
                nextSystemState = ClimberStates.IDLE;
            }
        }

        if (nextSystemState != systemState) {
            mStateStartTime = BreadUtil.getFPGATimeSeconds();
            systemState = nextSystemState;
        }
        double end = Logger.getInstance().getRealTimestamp();
        Logger.getInstance().recordOutput("LoggedRobot/ClimberPeriodicMs", (end-start)/1000);
    }    

    /** Requests the climber to deploy */
    public void requestDeploy() {
        requestDeploy = true;
        requestRun = false;
        requestIdle = false;
    }

    /** Requests the climber to idle */
    public void requestIdle() {
        requestDeploy = false;
        requestRun = false;
        requestIdle = true;
    }

    /** Requests the climber to run */
    public void requestRun(double openLoopRunPercent) {
        requestDeploy = false;
        requestRun = true;
        requestIdle = false;
        this.openLoopRunPercent = openLoopRunPercent;
    }

    /** Returns the system state of the climber */
    public ClimberStates getSystemState() {
        return systemState;
    }

    public boolean atSetpoint(double setpoint) {
        return BreadUtil.atReference(climberInputs.heightMeters, setpoint, CLIMBER_SETPOINT_TOLERANCE, true);
    }

    /** Returns the Error concentration for the following elevator motor */
    public double getClimberErrorConc(){
        return(climberErrCount/errCheckNum);
    }

    /** Resets error counters */
    public void resetError(){
        climberErrCount = 0;
        errCheckNum = 1;
    }  
    
}

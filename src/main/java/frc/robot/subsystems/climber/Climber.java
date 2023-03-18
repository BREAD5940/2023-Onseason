package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.BreadUtil;

import static frc.robot.Constants.Climber.*;

import com.ctre.phoenix.ErrorCode;

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

    /* Instantiate IO class in the constructor */
    public Climber(ClimberIO climberIO) {
        this.climberIO = climberIO;
    }

    /* Periodic method */
    @Override
    public void periodic() {
        climberIO.updateInputs(climberInputs);
         /* Fault Handling stuffs */
         if(climberInputs.lastClimberError != ErrorCode.OK.toString()){
            climberErrCount++;
        }
        climberIO.clearFault();

        Logger.getInstance().processInputs("Climber", climberInputs);

        ClimberStates nextSystemState = systemState;

        if (systemState == ClimberStates.RETRACTED) {
            climberIO.enableBrakeMode(true);
            // climberIO.setPercent(0.0);

            if (requestDeploy) {
                nextSystemState = ClimberStates.DEPLOYING;
            }
        } else if (systemState == ClimberStates.DEPLOYING) {
            climberIO.enableBrakeMode(true);
            climberIO.setHeight(CLIMBER_DEPLOY_HEIGHT);

            if (atSetpoint(CLIMBER_DEPLOY_HEIGHT)) {
                nextSystemState = ClimberStates.IDLE;
            }
        } else if (systemState == ClimberStates.IDLE) {
            climberIO.enableBrakeMode(true);
            climberIO.setPercent(0.0);

            if (requestRun) {
                nextSystemState = ClimberStates.RUNNING;
            }
        } else if (systemState == ClimberStates.RUNNING) {
            climberIO.enableBrakeMode(true);
            climberIO.setPercent(openLoopRunPercent);

            if (requestIdle) {
                nextSystemState = ClimberStates.IDLE;
            }
        }

        if (nextSystemState != systemState) {
            mStateStartTime = BreadUtil.getFPGATimeSeconds();
            systemState = nextSystemState;
        }
    }    

    public boolean atSetpoint(double setpoint) {
        return BreadUtil.atReference(climberInputs.heightMeters, setpoint, CLIMBER_SETPOINT_TOLERANCE, true);
    }

    /* Returns the Error concentration for the following elevator motor */
    public double getClimberErrorConc(){
        return(climberErrCount/errCheckNum);
    }

    /* Resets error counters */
    public void resetError(){
        climberErrCount = 0;
        errCheckNum = 1;
    }  
}

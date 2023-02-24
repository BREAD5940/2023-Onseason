package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import frc.robot.commons.BreadUtil;

import static frc.robot.Constants.Climber.*;

public class Climber {

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

    /* This onLoop() method is to be called periodically */
    public void onLoop() {
        climberIO.updateInputs(climberInputs);
        Logger.getInstance().processInputs("Climber", climberInputs);

        ClimberStates nextSystemState = systemState;

        if (systemState == ClimberStates.RETRACTED) {
            climberIO.enableBrakeMode(true);
            climberIO.setPercent(0.0);

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
    
}

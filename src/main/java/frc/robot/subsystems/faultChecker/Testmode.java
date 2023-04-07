package frc.robot.subsystems.faultChecker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;

public class Testmode {
    public Testmode() {
    }

    public enum State {
        PRE_START,
        SWERVE_DRIVE,
        SWERVE_STEER,
        SUPERSTRUCTURE_HOME,
        LED_DANCE
    }

    public State state = State.PRE_START;
    public boolean watingForStateAdvance = false;

    private int reusedLoopCounter = 0;

    public void resetState() {
        state = State.PRE_START;
    }

    public void periodic() {
        RobotContainer.leds.setDangerBlink(true);

        Logger.getInstance().recordOutput("TesterState", state.toString());

        /* State Transition */
        if (RobotContainer.driver.getLeftBumperPressed()) {
            state = State.values()[state.ordinal() + 1];
            reusedLoopCounter = 0;
            watingForStateAdvance = false;
        }

        if (RobotContainer.driver.getAButton()) {

            /* Swerve Drive Test */
            if (state == State.SWERVE_DRIVE) {
                RobotContainer.swerve.requestVelocity(new ChassisSpeeds(0.1, 0, 0), false, false);
            }

            /* Swerve Steer Test */
            if (state == State.SWERVE_STEER) {
                if (reusedLoopCounter % 50 > 25) {
                    RobotContainer.swerve.requestVelocity(new ChassisSpeeds(0.1, 0, 0), false, false);
                } else {
                    RobotContainer.swerve.requestVelocity(new ChassisSpeeds(0, 0.1, 0), false, false);
                }
                reusedLoopCounter++;
            }
            
            if (state != State.SWERVE_DRIVE && state != State.SWERVE_STEER){
                RobotContainer.swerve.requestVelocity(new ChassisSpeeds(0, 0, 0), false, false);
            }

            /* Homing test */
            if (state == State.SUPERSTRUCTURE_HOME) {
                if (!watingForStateAdvance) {
                    RobotContainer.superstructure.requestHome();
                    watingForStateAdvance = true;
                }
            }

            /* LED dance */
            if (state == State.LED_DANCE) {
                RobotContainer.leds.setDangerBlink(false);
                RobotContainer.leds.setHappyBlink(true);
            } else {
                RobotContainer.leds.setDangerBlink(true);
            }
        } else {
            RobotContainer.swerve.requestVelocity(new ChassisSpeeds(0, 0, 0), false, false);
        }
    }
}

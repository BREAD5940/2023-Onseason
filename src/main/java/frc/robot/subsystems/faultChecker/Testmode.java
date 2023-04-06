package frc.robot.subsystems.faultChecker;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;

public class Testmode {
    public Testmode(){}

    public enum State{
        PRE_START,
        SWERVE_DRIVE,
        SWERVE_STEER,

        SUPERSTRUCTURE_HOME,
        LED_DANCE
    }
    public State state = State.PRE_START;
    public boolean watingForStateAdvance = false;

    private int reusedLoopCounter = 0;

    public void periodic(){
        if(RobotContainer.driver.getAButton()){

            /*State Transition */
            if(RobotContainer.driver.getLeftBumper()){
                state = State.values()[state.ordinal() + 1];
                while(RobotContainer.driver.getLeftBumper()){}
                reusedLoopCounter = 0;
                watingForStateAdvance = false;
            }

            /*Swerve Drive Test */
            if(state == State.SWERVE_DRIVE){
                RobotContainer.swerve.requestVelocity(new ChassisSpeeds(0.1,0,0), false, false);
            } else {
                RobotContainer.swerve.requestVelocity(new ChassisSpeeds(0,0,0), false, false);
            }

            /*Swerve Steer Test */
            if(state == State.SWERVE_DRIVE){
                if(reusedLoopCounter % 50 > 25){
                    RobotContainer.swerve.requestVelocity(new ChassisSpeeds(0.1,0,0), false, false);
                } else {
                    RobotContainer.swerve.requestVelocity(new ChassisSpeeds(0,0.1,0), false, false);
                }
            } else {
                RobotContainer.swerve.requestVelocity(new ChassisSpeeds(0,0,0), false, false);
            }

            /*Homing test */
            if(state == State.SUPERSTRUCTURE_HOME){
                if(!watingForStateAdvance){
                    RobotContainer.superstructure.requestHome();
                    watingForStateAdvance = true;
                }
            }

            /*LED dance */
            if(state == State.LED_DANCE){
                RobotContainer.leds.setHappyBlink(true);
            } else {
                RobotContainer.leds.setDangerBlink(true);
            }
        }
    }
}

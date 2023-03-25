package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.BreadUtil;
import frc.robot.commons.LoggedTunableNumber;
import frc.robot.subsystems.elevatorarm.ArmIO;
import frc.robot.subsystems.elevatorarm.ElevatorArmLowLevel;
import frc.robot.subsystems.elevatorarm.ElevatorArmLowLevel.ElevatorArmSystemStates;
import frc.robot.subsystems.elevatorarm.ElevatorIO;
import frc.robot.subsystems.endeffector.BeamBreakIO;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorIO;
import frc.robot.subsystems.floorintake.FloorIntake;
import frc.robot.subsystems.floorintake.FloorIntake.FloorIntakeStates;
import frc.robot.subsystems.floorintake.FloorIntakeIO;

import static frc.robot.Constants.Elevator.*;
import static frc.robot.Constants.Arm.*;
import static frc.robot.Constants.FloorIntake.*;
import static frc.robot.Constants.EndEffector.*;

public class Superstructure extends SubsystemBase {

    /* IO and subsystems */
    private ElevatorArmLowLevel elevatorArmLowLevel;
    private EndEffector endEffector;
    private FloorIntake floorIntake;

    /* State variables */
    private double mStateStartTime = 0.0;
    private SuperstructureState systemState = SuperstructureState.PRE_HOME;
    private boolean requestHome = false;
    private boolean requestFloorIntakeCube = false;
    private boolean requestHPIntakeCube = false;
    private boolean requestHPIntakeCone = false;
    private boolean requestPreScore = false;
    private boolean requestScore = false;
    private boolean requestSpit = false;
    private boolean requestFloorIntakeCone = false;
    private Supplier<Double> floorIntakePressure = () -> 0.0;
    private Timer currentTriggerTimer = new Timer();
    private boolean currentTriggerTimerStarted = false;


    private Level level = Level.LOW;
    private GamePiece piece = GamePiece.CUBE;

    private boolean homedOnce = false;

    /* Presets */
    public static LoggedTunableNumber preCubeHighHeight = new LoggedTunableNumber("Elevator/PreCubeHighHeight", ELEVATOR_PRE_CUBE_HIGH);
    public static LoggedTunableNumber preConeHighHeight = new LoggedTunableNumber("Elevator/PreConeHighHeight", ELEVATOR_PRE_CONE_HIGH);
    public static LoggedTunableNumber coneSlamHeight = new LoggedTunableNumber("Elevator/ConeSlamHighHeight", ELEVATOR_CONE_SLAM_HIGH);
    public static LoggedTunableNumber conePulloutHeight = new LoggedTunableNumber("Elevator/ConePulloutHighHeight", ELEVATOR_CONE_PULL_OUT_HIGH);
    public static LoggedTunableNumber cubeOffset = new LoggedTunableNumber("Elevator/CubeOffsetHeight", ELEVATOR_CUBE_OFFSET);
    public static LoggedTunableNumber coneOffset = new LoggedTunableNumber("Elevator/ConeOffsetHeight", ELEVATOR_CONE_OFFSET);
    public static LoggedTunableNumber preLowHeight = new LoggedTunableNumber("Elevator/PreLowHeight", ELEVATOR_PRE_LOW);
    public static LoggedTunableNumber floorIntakeCubeHeight = new LoggedTunableNumber("Elevator/FloorIntakeCubeHeight", ELEVATOR_FLOOR_INTAKE_CUBE);
    public static LoggedTunableNumber doubleSubstationConeHeight = new LoggedTunableNumber("Elevator/DoubleSubstationConeHeight", ELEVATOR_DOUBLE_SUBSTATION_CONE);
    public static LoggedTunableNumber doubleSubstationCubeHeight = new LoggedTunableNumber("Elevator/DoubleSubstationCubeHeight", ELEVATOR_DOUBLE_SUBSTATION_CUBE);

    public static LoggedTunableNumber cubeAngle = new LoggedTunableNumber("Arm/ArmCubeAngle", ARM_PRE_SCORE_CUBE);
    public static LoggedTunableNumber coneAngle = new LoggedTunableNumber("Arm/ArmConeAngle", ARM_PRE_SCORE_CONE);
    public static LoggedTunableNumber slamConeAngle = new LoggedTunableNumber("Arm/ArmSlamConeAngle", ARM_SLAM_CONE);
    public static LoggedTunableNumber lowAngle = new LoggedTunableNumber("Arm/ArmLowAngle", ARM_PRE_SCORE_LOW);
    public static LoggedTunableNumber floorIntakeCubeAngle = new LoggedTunableNumber("Arm/ArmFloorIntakeCube", ARM_FLOOR_INTAKE_CUBE);
    public static LoggedTunableNumber doubleSubstationConeAngle = new LoggedTunableNumber("Arm/ArmDoubleSubstationConeAngle", ARM_DOUBLE_SUBSTATION_CONE);
    public static LoggedTunableNumber doubleSubstationCubeAngle = new LoggedTunableNumber("Arm/ArmDoubleSubstationCubeAngle", ARM_DOUBLE_SUBSTATION_CUBE);
    // TunableNumber floorIntakeOutput = new TunableNumber("FloorIntake/Output", 0.0);

    double lastFPGATimestamp = 0.0;

    /* System States */
    public enum SuperstructureState {
        PRE_HOME,
        HOMING,
        IDLE,
        FLOOR_INTAKE_CUBE,
        HP_INTAKE_CUBE,
        HP_INTAKE_CONE,
        HP_INTAKE_CONE_INTER,
        PREPARE_TO_SPIT,
        SPIT,
        EXIT_SPIT,
        PRE_PLACE_PIECE_LOW,
        PRE_PLACE_CUBE,
        PRE_PLACE_CONE,
        EXHAUSTING_PIECE_LOW,
        EXHAUSTING_CUBE,
        SLAM_CONE, 
        PULL_OUT_CONE,
        FLOOR_INTAKE_CONE_A, 
        FLOOR_INTAKE_CONE_B, 
        FLOOR_INTAKE_CONE_C
    }

    public enum Level {
        LOW, MID, HIGH
    }

    public enum GamePiece {
        CONE, CUBE
    }

    /** Set IO and subsystems to what they should be equal to */
    public Superstructure(ElevatorIO elevatorIO, ArmIO armIO, EndEffectorIO endEffectorIO, FloorIntakeIO floorIntakeIO) {
        elevatorArmLowLevel = new ElevatorArmLowLevel(armIO, elevatorIO);
        endEffector = new EndEffector(endEffectorIO);
        floorIntake = new FloorIntake(floorIntakeIO);
    }

    @Override
    public void periodic() {

        /* Logs */
        Logger.getInstance().recordOutput("SuperstructureState", systemState.toString());
        Logger.getInstance().recordOutput("Superstructure/loopCycleTime", Logger.getInstance().getRealTimestamp()/1.0E6 - lastFPGATimestamp);
        lastFPGATimestamp = Logger.getInstance().getRealTimestamp()/1.0E6;

        /* On loops */
        elevatorArmLowLevel.onLoop();
        endEffector.onLoop();
        floorIntake.onLoop();

        /* Statemachine things */
        SuperstructureState nextSystemState = systemState;
        if (systemState == SuperstructureState.PRE_HOME) {
            // Outputs
            endEffector.idling();

            // Transitions
            if (requestHome) {
                elevatorArmLowLevel.requestHome();
                floorIntake.requestHome();
                nextSystemState = SuperstructureState.HOMING;
            }
        } else if (systemState == SuperstructureState.HOMING) {
            // Outputs
            endEffector.idling();

            // Transitions 
            if (elevatorArmLowLevel.getSystemState() == ElevatorArmSystemStates.IDLE && floorIntake.getSystemState() == FloorIntakeStates.IDLE) {
                nextSystemState = SuperstructureState.IDLE;
                requestHome = false;
                homedOnce = true;
            }
        } else if (systemState == SuperstructureState.IDLE) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(ELEVATOR_IDLE_POSE, ARM_IDLE_POSE);
            if (elevatorArmLowLevel.atElevatorSetpoint(ELEVATOR_IDLE_POSE) && elevatorArmLowLevel.atArmSetpoint(ARM_IDLE_POSE)) {
                endEffector.holdGamePiece();
            }
            floorIntake.requestClosedLoop(0.0, INTAKE_IDLE_POSITION);
            currentTriggerTimerStarted = false;

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (requestFloorIntakeCube) {
                nextSystemState = SuperstructureState.FLOOR_INTAKE_CUBE;
            } else if (requestHPIntakeCone) {
                nextSystemState = SuperstructureState.HP_INTAKE_CONE;
            } else if (requestHPIntakeCube) {
                nextSystemState = SuperstructureState.HP_INTAKE_CUBE;
            } else if (requestSpit) {
                nextSystemState = SuperstructureState.PREPARE_TO_SPIT;
            } else if (requestPreScore && level == Level.LOW) {
                nextSystemState = SuperstructureState.PRE_PLACE_PIECE_LOW;
            } else if (requestPreScore && piece == GamePiece.CUBE) {
                nextSystemState = SuperstructureState.PRE_PLACE_CUBE;
            } else if (requestPreScore && piece == GamePiece.CONE) {
                nextSystemState = SuperstructureState.PRE_PLACE_CONE;
            } else if (requestFloorIntakeCone) {
                nextSystemState = SuperstructureState.FLOOR_INTAKE_CONE_A;
            }
        } else if (systemState == SuperstructureState.FLOOR_INTAKE_CUBE) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(floorIntakeCubeHeight.get(), floorIntakeCubeAngle.get());
            endEffector.intakeCube();
            floorIntake.requestClosedLoop(0.7, 144.0 + floorIntakePressure.get() * 20.0);

            if (endEffector.getStatorCurrent() > INTAKE_CUBE_CURR_LIMIT && !currentTriggerTimerStarted) {
                currentTriggerTimerStarted = true;
                currentTriggerTimer.reset();
                currentTriggerTimer.start();
            } 

            if (endEffector.getStatorCurrent() < INTAKE_CUBE_CURR_LIMIT) {
                currentTriggerTimer.reset();
                currentTriggerTimer.stop();
                currentTriggerTimerStarted = false;
            }

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (!requestFloorIntakeCube) {
                nextSystemState = SuperstructureState.IDLE;
            } else if (currentTriggerTimer.get() > 0.05) {
                nextSystemState = SuperstructureState.IDLE;
                requestFloorIntakeCube = false;
            }
        } else if (systemState == SuperstructureState.HP_INTAKE_CONE) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(doubleSubstationConeHeight.get(), doubleSubstationConeAngle.get());
            endEffector.intakeCone();
            floorIntake.requestClosedLoop(0.0, INTAKE_IDLE_POSITION);

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (!requestHPIntakeCone) {
                nextSystemState = SuperstructureState.HP_INTAKE_CONE_INTER;
            } else if (endEffector.getStatorCurrent() > INTAKE_CONE_CURR_LIMIT) {
                requestHPIntakeCone = false;
                nextSystemState = SuperstructureState.HP_INTAKE_CONE_INTER;
            }
        } else if (systemState == SuperstructureState.HP_INTAKE_CONE_INTER) {
            elevatorArmLowLevel.requestDesiredState(doubleSubstationConeHeight.get(), 90.0);
            endEffector.intakeCone();
            floorIntake.requestClosedLoop(0.0, INTAKE_IDLE_POSITION);

            if (elevatorArmLowLevel.getState()[1] < 120.0) {
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.HP_INTAKE_CUBE) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(doubleSubstationCubeHeight.get(), doubleSubstationCubeAngle.get());
            endEffector.intakeCube();
            floorIntake.requestClosedLoop(0.0, INTAKE_IDLE_POSITION);

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (!requestHPIntakeCube) {
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.SPIT) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(Units.inchesToMeters(-2.0), ARM_UNJAM_POSITION);
            endEffector.idling();
            floorIntake.requestClosedLoop(-1.0, INTAKE_UNJAM_POSITION);

            // Transitions
            if (!requestSpit) {
                nextSystemState = SuperstructureState.EXIT_SPIT;
            }
        } else if (systemState == SuperstructureState.PREPARE_TO_SPIT) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(ELEVATOR_IDLE_POSE, ARM_UNJAM_POSITION);
            endEffector.idling();
            floorIntake.requestClosedLoop(-1.0, INTAKE_UNJAM_POSITION);

            // Transitions
            if (!requestSpit) {
                nextSystemState = SuperstructureState.IDLE;
            } else if (elevatorArmLowLevel.atArmSetpoint(ARM_UNJAM_POSITION)) {
                nextSystemState = SuperstructureState.SPIT;
            }
        } else if (systemState == SuperstructureState.EXIT_SPIT) {
             // Outputs
            elevatorArmLowLevel.requestDesiredState(ELEVATOR_IDLE_POSE, ARM_UNJAM_POSITION);
            endEffector.idling();
            floorIntake.requestClosedLoop(-1.0, INTAKE_UNJAM_POSITION);

            // Transitions
            if (elevatorArmLowLevel.atElevatorSetpoint(ELEVATOR_IDLE_POSE) || BreadUtil.getFPGATimeSeconds() - mStateStartTime >= 2.0) {
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.PRE_PLACE_PIECE_LOW) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(preLowHeight.get(), lowAngle.get());
            endEffector.holdGamePiece();
            floorIntake.requestClosedLoop(0.0, INTAKE_IDLE_POSITION);

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (requestScore) {
                nextSystemState = SuperstructureState.EXHAUSTING_PIECE_LOW;
            } else if (!requestPreScore || level != Level.LOW) {
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.PRE_PLACE_CUBE) {
            // Outputs
            if (level == Level.MID) {
                elevatorArmLowLevel.requestDesiredState(preCubeHighHeight.get() - cubeOffset.get(), cubeAngle.get());
            } else {
                elevatorArmLowLevel.requestDesiredState(preCubeHighHeight.get(), cubeAngle.get());
            }
            endEffector.holdGamePiece();
            floorIntake.requestClosedLoop(0.0, INTAKE_IDLE_POSITION);

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (requestScore) {
                nextSystemState = SuperstructureState.EXHAUSTING_CUBE;
            } else if (!requestPreScore || level == Level.LOW || piece == GamePiece.CONE) {
                nextSystemState = SuperstructureState.IDLE;
            }

        } else if (systemState == SuperstructureState.PRE_PLACE_CONE) {
            // Outputs
            if (level == Level.MID) {
                elevatorArmLowLevel.requestDesiredState(preConeHighHeight.get() - coneOffset.get(), coneAngle.get());
            } else {
                elevatorArmLowLevel.requestDesiredState(preConeHighHeight.get(), coneAngle.get());
            }
            endEffector.holdConeElevatorMoving();
            floorIntake.requestClosedLoop(0.0, INTAKE_IDLE_POSITION);

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (requestScore) {
                nextSystemState = SuperstructureState.SLAM_CONE;
            } else if (!requestPreScore || level == Level.LOW || piece == GamePiece.CUBE) {
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.EXHAUSTING_PIECE_LOW) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(preLowHeight.get(), lowAngle.get());
            endEffector.spitCube();
            floorIntake.requestClosedLoop(0.0, INTAKE_IDLE_POSITION);

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (!requestScore) {
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.EXHAUSTING_CUBE) {
            // Outputs
            if (level == Level.MID) {
                elevatorArmLowLevel.requestDesiredState(preCubeHighHeight.get() - cubeOffset.get(), cubeAngle.get());
            } else {
                elevatorArmLowLevel.requestDesiredState(preCubeHighHeight.get(), cubeAngle.get());
            }
            endEffector.spitCube();
            floorIntake.requestClosedLoop(0.0, INTAKE_IDLE_POSITION);

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (!requestScore) {
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.SLAM_CONE) {
            // Outputs
            if (level == Level.MID) {
                elevatorArmLowLevel.requestDesiredState(coneSlamHeight.get() - coneOffset.get(), slamConeAngle.get());
            } else {
                elevatorArmLowLevel.requestDesiredState(coneSlamHeight.get(), slamConeAngle.get());
            }
            endEffector.enableBrakeMode(false);
            endEffector.holdConeElevatorMoving();
            floorIntake.requestClosedLoop(0.0, INTAKE_IDLE_POSITION);

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (BreadUtil.getFPGATimeSeconds() - mStateStartTime > 0.5) {
                nextSystemState = SuperstructureState.PULL_OUT_CONE;
            }
        } else if (systemState == SuperstructureState.PULL_OUT_CONE) {
            // Outputs
            if (level == Level.MID) {
                elevatorArmLowLevel.requestDesiredState(conePulloutHeight.get() - coneOffset.get(), slamConeAngle.get());
            } else {
                elevatorArmLowLevel.requestDesiredState(conePulloutHeight.get(), slamConeAngle.get());
            }
            endEffector.idling();
            floorIntake.requestClosedLoop(0.0, INTAKE_IDLE_POSITION);

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (!requestScore) {
                nextSystemState = SuperstructureState.IDLE;
                endEffector.holdConeElevatorMoving();
            }
        } else if (systemState == SuperstructureState.FLOOR_INTAKE_CONE_A) {
            // Outputs
            endEffector.idling();
            if (floorIntake.getRollerCurrent() > 49.0) {
                floorIntake.requestClosedLoop(-0.75, 7.0);
            } else {
                floorIntake.requestClosedLoop(-0.75, 155.0);
            }
            elevatorArmLowLevel.requestDesiredState(ELEVATOR_IDLE_POSE, ARM_IDLE_POSE);

            // Transitions
            if (!requestFloorIntakeCone) {
                nextSystemState = SuperstructureState.IDLE;
            } else if (floorIntake.getAngle() < 120.0 && floorIntake.getRollerCurrent() > 49.0) {
                System.out.println(floorIntake.getAngle() + " " + floorIntake.getRollerCurrent());
                nextSystemState = SuperstructureState.FLOOR_INTAKE_CONE_B;
            }
        } else if (systemState == SuperstructureState.FLOOR_INTAKE_CONE_B) {
            // Outputs
            if (elevatorArmLowLevel.getState()[0] > 0.4) {
                elevatorArmLowLevel.requestDesiredState(0.5, 2.0);
            } else {
                elevatorArmLowLevel.requestDesiredState(0.5, 90.0);
            }
            floorIntake.requestClosedLoop(-0.75, 7.0);
            endEffector.intakeCone();

            // Transitions
            if (elevatorArmLowLevel.atArmSetpoint(9.0) && elevatorArmLowLevel.atElevatorSetpoint(0.5)) {
                nextSystemState = SuperstructureState.FLOOR_INTAKE_CONE_C;
            } else if (!requestFloorIntakeCone) {
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.FLOOR_INTAKE_CONE_C) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(0.25, 9.0);
            if (elevatorArmLowLevel.atElevatorSetpoint(0.25)) {
                floorIntake.requestClosedLoop(0.25, 0.0);
            } else {
                floorIntake.requestClosedLoop(-0.75, 7.0);
            }
            endEffector.intakeCone();

            if (endEffector.isHoldingCone() && !currentTriggerTimerStarted) { //TODO: check if we still need currentTriggerTimer
                currentTriggerTimerStarted = true;
                currentTriggerTimer.reset();
                currentTriggerTimer.start();
            } 

            if (!endEffector.isHoldingCone()) {
                currentTriggerTimer.reset();
                currentTriggerTimer.stop();
                currentTriggerTimerStarted = false;
            }

            // Transitions
            if (!requestFloorIntakeCone) {
                nextSystemState = SuperstructureState.IDLE;
            } else if (currentTriggerTimer.get() > 0.25) {
                nextSystemState = SuperstructureState.IDLE;
                requestFloorIntakeCone = false;
            }
        }

        if (nextSystemState != systemState) {
            systemState = nextSystemState;
            mStateStartTime = BreadUtil.getFPGATimeSeconds();
        }


    } 

    /** Requests the entire system to home */
    public void requestHome() {
        System.out.println("Requested the superstructure to home!");
        requestHome = true;
        unsetAllRequests();
    } 

    /** Requests the entire system to go into its idling mode */
    public void requestIdle() {
        unsetAllRequests();
    }

    /** Requests the system to floor intake a cube */
    public void requestFloorIntakeCube(Supplier<Double> floorIntakePressure) {
        unsetAllRequests();
        requestFloorIntakeCube = true;
        this.floorIntakePressure = floorIntakePressure;
    }

    /** Requests the system to spit */
    public void requestSpit() {
        unsetAllRequests();
        requestSpit = true;
    }

    /** Requests the sytem to intake a cone from the double substation */
    public void requestIntakeConeDoubleSubstation() {
        unsetAllRequests();
        requestHPIntakeCone = true;
    }

    /** Requests the sytem to intake a cube from the double substation */
    public void requestIntakeCubeDoubleSubstation() {
        unsetAllRequests();
        requestHPIntakeCube = true;
    }

    /** Requests the system to pre score */
    public void requestPreScore(Level level, GamePiece piece) {
        unsetAllRequests();
        requestPreScore = true;
        this.level = level;
        this.piece = piece;
    }

    /** Requests the system to score */
    public void requestScore() {
        unsetAllRequests();
        requestScore = true;
    }

    /** Requests the system to floor intake a cone */
    public void requestFloorIntakeCone() {
        unsetAllRequests();
        requestFloorIntakeCone = true;
    }
    
    /** Sets all of the requests to false */
    private void unsetAllRequests() {
        requestFloorIntakeCube = false;
        requestHPIntakeCube = false;
        requestHPIntakeCone = false;
        requestPreScore = false;
        requestScore = false;
        requestSpit = false;
        requestFloorIntakeCone = false;
    }

    /** Zeroes all sensors */
    public void zeroSensors() {
        elevatorArmLowLevel.zeroSensors();
    }

    /** Returns the system state */
    public SuperstructureState getSystemState() {
        return systemState;
    }

    /** Returns whether or not the elevator is at a certain height */
    public boolean atElevatorSetpoint(double height) {
        return elevatorArmLowLevel.atElevatorSetpoint(height);
    }
    
    /** Returns the height of the elevator */
    public double getElevatorHeight() {
        return elevatorArmLowLevel.elevatorInputs.posMeters;
    }

    /** Returns whether or not the superstructure has homed once */
    public boolean homedOnce() {
        return homedOnce;
    }

     /** Returns the error concentration for the arm motor */
     public double getArmErrorConc(){
        return(elevatorArmLowLevel.getArmErrorConc());
    }

    /** Returns the error concentration for the arm encoder */
    public double getArmAzimuthErrorConc(){
        return(elevatorArmLowLevel.getArmAzimuthErrorConc());
    }

    /** Returns the error concentration for the leading elevator motor */
    public double getLeaderErrorConc(){
        return(elevatorArmLowLevel.getLeaderErrorConc());
    }

    /** Returns the Error concentration for the following elevator motor */
    public double getFollowerErrorConc(){
        return(elevatorArmLowLevel.getFollowerErrorConc());
    }

    /** Returns the Error concentration for the intake roller motor */
    public double getRollerErrorConc(){
        return(floorIntake.getRollerErrorConc());
    }

    /** Returns the Error concentration for the intake deploy motor */
    public double getDeployErrorConc(){
        return(floorIntake.getDeployErrorConc());
    }

    /** Returns the Error concentration for the end effector motor */
    public double getEndEffectorErrorConc(){
        return(endEffector.getEndEffectorErrorConc());
    }

    /** Resets error counters */
    public void resetError(){
        elevatorArmLowLevel.resetError();
        floorIntake.resetError();
        endEffector.resetError();
    }

}


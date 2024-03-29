package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.BreadUtil;
import frc.robot.commons.LoggedTunableNumber;
import frc.robot.subsystems.elevatorarm.ArmIO;
import frc.robot.subsystems.elevatorarm.ElevatorArmLowLevel;
import frc.robot.subsystems.elevatorarm.ElevatorArmLowLevel.ElevatorArmSystemStates;
import frc.robot.subsystems.elevatorarm.ElevatorIO;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorIO;
import frc.robot.subsystems.floorintake.FloorIntake;
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
    private boolean requestSingleSubstationCone = false;
    private boolean requestHPIntakeCone = false;
    private boolean requestPreScore = false;
    private boolean requestScore = false;
    private boolean requestSpit = false;
    private boolean requestFloorIntakeCone = false;
    private boolean requestThrow = false;
    private boolean requestSpitCubeFront = false;
    private Supplier<Double> floorIntakePressure = () -> 0.0;
    private Timer currentTriggerTimer = new Timer();
    private Timer beamBreakTriggerTimer = new Timer();
    private boolean beamBreakTriggerTimeStarted = false;
    private boolean currentTriggerTimerStarted = false;
    private boolean goSlow = false;
    private double throwSetpoint = 0.0;


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
    public static LoggedTunableNumber singleSubstationConeHeight = new LoggedTunableNumber("Elevator/DoubleSubstationCubeHeight", ELEVATOR_SINGLE_SUBSTATION_CONE);

    public static LoggedTunableNumber cubeAngle = new LoggedTunableNumber("Arm/ArmCubeAngle", ARM_PRE_SCORE_CUBE);
    public static LoggedTunableNumber coneAngle = new LoggedTunableNumber("Arm/ArmConeAngle", ARM_PRE_SCORE_CONE);
    public static LoggedTunableNumber slamConeAngle = new LoggedTunableNumber("Arm/ArmSlamConeAngle", ARM_SLAM_CONE);
    public static LoggedTunableNumber lowAngle = new LoggedTunableNumber("Arm/ArmLowAngle", ARM_PRE_SCORE_LOW);
    public static LoggedTunableNumber floorIntakeCubeAngle = new LoggedTunableNumber("Arm/ArmFloorIntakeCube", ARM_FLOOR_INTAKE_CUBE);
    public static LoggedTunableNumber doubleSubstationConeAngle = new LoggedTunableNumber("Arm/ArmDoubleSubstationConeAngle", ARM_DOUBLE_SUBSTATION_CONE);
    public static LoggedTunableNumber singleSubstationConeAngle = new LoggedTunableNumber("Arm/ArmDoubleSubstationCubeAngle", ARM_SINGLE_SUBSTATION_CONE);
    public static LoggedTunableNumber spitAngle = new LoggedTunableNumber("Arm/spitAngle", 100.0);
    // TunableNumber floorIntakeOutput = new TunableNumber("FloorIntake/Output", 0.0);

    double lastFPGATimestamp = 0.0;

    /* System States */
    public enum SuperstructureState {
        PRE_HOME,
        HOMING,
        IDLE,
        PREPARE_TO_THROW, 
        THROWING, 
        FLOOR_INTAKE_CUBE,
        SINGLE_SUBSTATION_CONE,
        HP_INTAKE_CONE,
        HP_INTAKE_CONE_INTER,
        SPIT_CUBE_FRONT,
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

    /* Set IO and subsystems to what they should be equal to  */
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
                nextSystemState = SuperstructureState.HOMING;
            }
        } else if (systemState == SuperstructureState.HOMING) {
            // Outputs
            endEffector.idling();
            floorIntake.requestClosedLoop(0.0, INTAKE_IDLE_POSITION);

            // Transitions 
            if (elevatorArmLowLevel.getSystemState() == ElevatorArmSystemStates.IDLE) {
                nextSystemState = SuperstructureState.IDLE;
                requestHome = false;
                homedOnce = true;
            }
        } else if (systemState == SuperstructureState.IDLE) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(ELEVATOR_IDLE_POSE, ARM_IDLE_POSE, goSlow);
            if (elevatorArmLowLevel.atElevatorSetpoint(ELEVATOR_IDLE_POSE) && elevatorArmLowLevel.atArmSetpoint(ARM_IDLE_POSE)) {
                if (endEffector.getBeamBreakTriggered()) {
                    endEffector.holdCone();
                } else {
                    endEffector.holdCube();
                }
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
            } else if (requestSingleSubstationCone) {
                nextSystemState = SuperstructureState.SINGLE_SUBSTATION_CONE;
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
            } else if (requestThrow) {
                nextSystemState = SuperstructureState.PREPARE_TO_THROW;
            } else if (requestSpitCubeFront) {
                nextSystemState = SuperstructureState.SPIT_CUBE_FRONT;
            }
        } else if (systemState == SuperstructureState.PREPARE_TO_THROW) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(ELEVATOR_IDLE_POSE, ARM_IDLE_POSE, goSlow);
            floorIntake.requestClosedLoop(0.0, INTAKE_IDLE_POSITION + 20.0);
            endEffector.holdCube();

            // Transitions
            if (!requestThrow) {
                nextSystemState = SuperstructureState.IDLE;
            } else if (BreadUtil.atReference(elevatorArmLowLevel.getState()[1], ARM_IDLE_POSE, 45.0, true) && elevatorArmLowLevel.atElevatorSetpoint(ELEVATOR_IDLE_POSE)) {
                nextSystemState = SuperstructureState.THROWING;
            }
        } else if (systemState == SuperstructureState.THROWING) {
            // Outputs
            floorIntake.requestClosedLoop(0.0, INTAKE_IDLE_POSITION + 20.0);
            // if (elevatorArmLowLevel.getState()[1] > 100.0) {
            elevatorArmLowLevel.requestDesiredState(throwSetpoint, ARM_POST_THROW, goSlow);
            // } else {
            //     elevatorArmLowLevel.requestDesiredState(ELEVATOR_IDLE_POSE, ARM_POST_THROW, goSlow);
            // }  

            if (elevatorArmLowLevel.getState()[1] > 95.0 && elevatorArmLowLevel.getState()[0] > ELEVATOR_IDLE_POSE + Units.inchesToMeters(1.0)) {
                endEffector.throwCube();
            } else {
                endEffector.holdCube();
            }
                   
            // Transitions
            if (!requestThrow) {
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.FLOOR_INTAKE_CUBE) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(floorIntakeCubeHeight.get(), floorIntakeCubeAngle.get(), goSlow);
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
            } else if (currentTriggerTimer.get() > 0.05 && BreadUtil.getFPGATimeSeconds() - mStateStartTime >= 0.25 && endEffector.getMotorRPM() > -200.0) {
                nextSystemState = SuperstructureState.IDLE;
                requestFloorIntakeCube = false;
            }
        } else if (systemState == SuperstructureState.HP_INTAKE_CONE) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(doubleSubstationConeHeight.get(), doubleSubstationConeAngle.get(), goSlow);
            endEffector.intakeCone();
            floorIntake.requestClosedLoop(0.0, INTAKE_IDLE_POSITION);

            if (endEffector.getBeamBreakTriggered() && !beamBreakTriggerTimeStarted) {
                beamBreakTriggerTimeStarted = true;
                beamBreakTriggerTimer.reset();
                beamBreakTriggerTimer.start();
            } 

            if (!endEffector.getBeamBreakTriggered()) {
                beamBreakTriggerTimer.reset();
                beamBreakTriggerTimer.stop();
                beamBreakTriggerTimeStarted = false;
            }

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (!requestHPIntakeCone) {
                nextSystemState = SuperstructureState.HP_INTAKE_CONE_INTER;
            } else if (beamBreakTriggerTimer.get() > 0.1) {
                requestHPIntakeCone = false;
                nextSystemState = SuperstructureState.HP_INTAKE_CONE_INTER;
            }
        } else if (systemState == SuperstructureState.HP_INTAKE_CONE_INTER) {
            elevatorArmLowLevel.requestDesiredState(doubleSubstationConeHeight.get(), 90.0, goSlow);
            endEffector.intakeCone();
            floorIntake.requestClosedLoop(0.0, INTAKE_IDLE_POSITION);

            nextSystemState = SuperstructureState.IDLE;
        } else if (systemState == SuperstructureState.SINGLE_SUBSTATION_CONE) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(singleSubstationConeHeight.get(), singleSubstationConeAngle.get(), goSlow);
            endEffector.intakeCone();
            floorIntake.requestClosedLoop(0.0, INTAKE_IDLE_POSITION);

            if (endEffector.getBeamBreakTriggered() && !beamBreakTriggerTimeStarted) {
                beamBreakTriggerTimeStarted = true;
                beamBreakTriggerTimer.reset();
                beamBreakTriggerTimer.start();
            } 

            if (!endEffector.getBeamBreakTriggered()) {
                beamBreakTriggerTimer.reset();
                beamBreakTriggerTimer.stop();
                beamBreakTriggerTimeStarted = false;
            }

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (!requestSingleSubstationCone) {
                nextSystemState = SuperstructureState.IDLE;
            } else if (beamBreakTriggerTimer.get() > 0.1) {
                requestSingleSubstationCone = false;
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.SPIT) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(Units.inchesToMeters(-2.0), ARM_UNJAM_POSITION, goSlow);
            endEffector.idling();
            floorIntake.requestClosedLoop(-1.0, INTAKE_UNJAM_POSITION);

            // Transitions
            if (!requestSpit || BreadUtil.getFPGATimeSeconds() - mStateStartTime >= 1.5) {
                nextSystemState = SuperstructureState.EXIT_SPIT;
                requestSpit = false;
            }
        } else if (systemState == SuperstructureState.PREPARE_TO_SPIT) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(ELEVATOR_SPIT_POSE, ARM_UNJAM_POSITION, goSlow);
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
            elevatorArmLowLevel.requestDesiredState(ELEVATOR_IDLE_POSE, ARM_UNJAM_POSITION, goSlow);
            endEffector.idling();
            floorIntake.requestClosedLoop(-1.0, INTAKE_UNJAM_POSITION);

            // Transitions
            if (elevatorArmLowLevel.atElevatorSetpoint(ELEVATOR_IDLE_POSE) || BreadUtil.getFPGATimeSeconds() - mStateStartTime >= 2.0) {
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.PRE_PLACE_PIECE_LOW) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(preLowHeight.get(), lowAngle.get(), goSlow);
            if (endEffector.getBeamBreakTriggered()) {
                endEffector.holdCone();
            } else {
                endEffector.holdCube();
            }
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
                elevatorArmLowLevel.requestDesiredState(preCubeHighHeight.get() - cubeOffset.get(), cubeAngle.get(), goSlow);
            } else {
                elevatorArmLowLevel.requestDesiredState(preCubeHighHeight.get(), cubeAngle.get(), goSlow);
            }
            endEffector.holdCube();
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
                elevatorArmLowLevel.requestDesiredState(preConeHighHeight.get() - coneOffset.get(), coneAngle.get(), goSlow);
            } else {
                elevatorArmLowLevel.requestDesiredState(preConeHighHeight.get(), coneAngle.get(), goSlow);
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
            elevatorArmLowLevel.requestDesiredState(preLowHeight.get(), lowAngle.get(), goSlow);
            endEffector.spitLow();
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
                elevatorArmLowLevel.requestDesiredState(preCubeHighHeight.get() - cubeOffset.get(), cubeAngle.get(), goSlow);
            } else {
                elevatorArmLowLevel.requestDesiredState(preCubeHighHeight.get(), cubeAngle.get(), goSlow);
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
                elevatorArmLowLevel.requestDesiredState(coneSlamHeight.get() - coneOffset.get(), slamConeAngle.get(), goSlow);
            } else {
                elevatorArmLowLevel.requestDesiredState(coneSlamHeight.get(), slamConeAngle.get(), goSlow);
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
                elevatorArmLowLevel.requestDesiredState(conePulloutHeight.get() - coneOffset.get(), slamConeAngle.get(), goSlow);
            } else {
                elevatorArmLowLevel.requestDesiredState(conePulloutHeight.get(), slamConeAngle.get(), goSlow);
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
            if (floorIntake.getRollerCurrent() > 55.0) {
                floorIntake.requestClosedLoop(-0.75, -2.373046875);
            } else {
                floorIntake.requestClosedLoop(-0.75, 154.0);
            }
            elevatorArmLowLevel.requestDesiredState(ELEVATOR_IDLE_POSE, ARM_IDLE_POSE, goSlow);

            // Transitions
            if (!requestFloorIntakeCone) {
                nextSystemState = SuperstructureState.IDLE;
            } else if (floorIntake.getAngle() < 120.0 && floorIntake.getRollerRPM() > -500.0 && BreadUtil.getFPGATimeSeconds() - mStateStartTime >= 0.5) {
                nextSystemState = SuperstructureState.FLOOR_INTAKE_CONE_B;
            }
        } else if (systemState == SuperstructureState.FLOOR_INTAKE_CONE_B) {
            // Outputs
            if (elevatorArmLowLevel.getState()[0] > 0.4) {
                elevatorArmLowLevel.requestDesiredState(0.5, 24.67578125, goSlow);
            } else {
                elevatorArmLowLevel.requestDesiredState(0.5, 90.0, goSlow);
            }
            floorIntake.requestClosedLoop(-0.75, -2.373046875);
            endEffector.intakeCone();

            // Transitions
            if (elevatorArmLowLevel.atArmSetpoint(24.67578125) && elevatorArmLowLevel.atElevatorSetpoint(0.5)) {
                nextSystemState = SuperstructureState.FLOOR_INTAKE_CONE_C;
            } else if (!requestFloorIntakeCone) {
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.FLOOR_INTAKE_CONE_C) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(0.15, 24.67578125, goSlow);
            if (elevatorArmLowLevel.atElevatorSetpoint(0.15)) {
                floorIntake.requestClosedLoop(0.15, 0.0);
            } else {
                floorIntake.requestClosedLoop(-0.75, -2.373046875);
            }
            endEffector.intakeCone();

 
            if (endEffector.getBeamBreakTriggered() && !beamBreakTriggerTimeStarted) {
                beamBreakTriggerTimeStarted = true;
                beamBreakTriggerTimer.reset();
                beamBreakTriggerTimer.start();
            } 

            if (!endEffector.getBeamBreakTriggered()) {
                beamBreakTriggerTimer.reset();
                beamBreakTriggerTimer.stop();
                beamBreakTriggerTimeStarted = false;
            }

            // Transitions
            if (!requestFloorIntakeCone) {
                nextSystemState = SuperstructureState.IDLE;
            } else if (beamBreakTriggerTimer.get() > 0.1) {
                nextSystemState = SuperstructureState.IDLE;
                requestFloorIntakeCone = false;
            }
        } else if (systemState == SuperstructureState.SPIT_CUBE_FRONT) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(floorIntakeCubeHeight.get(), floorIntakeCubeAngle.get(), false);
            if (elevatorArmLowLevel.atArmSetpoint(floorIntakeCubeAngle.get()) && elevatorArmLowLevel.atElevatorSetpoint(floorIntakeCubeHeight.get())) {
                endEffector.spitCube();
            } else {
                endEffector.holdCube();
            }
            floorIntake.requestClosedLoop(-0.5, 134.0);

            // Transitions
            if (!requestSpitCubeFront) {
                nextSystemState = SuperstructureState.IDLE;
            } 
        }

        if (nextSystemState != systemState) {
            systemState = nextSystemState;
            mStateStartTime = BreadUtil.getFPGATimeSeconds();
        }


    } 

    /* Requests the entire system to home */
    public void requestHome() {
        requestHome = true;
        unsetAllRequests();
    } 

    /* Requests the entire system to go into its idling mode */
    public void requestIdle() {
        unsetAllRequests();
    }

    /* Requests the system to floor intake a cube */
    public void requestFloorIntakeCube(Supplier<Double> floorIntakePressure) {
        unsetAllRequests();
        requestFloorIntakeCube = true;
        this.floorIntakePressure = floorIntakePressure;
    }

    /* Requests the system to spit */
    public void requestSpit() {
        unsetAllRequests();
        requestSpit = true;
    }

    /* Requests the system to intake a cone from the double substation */
    public void requestIntakeConeDoubleSubstation() {
        unsetAllRequests();
        requestHPIntakeCone = true;
    }

    /* Requests the system to spit a cube from the front */
    public void requestSpitCubeFront() {
        unsetAllRequests();
        requestSpitCubeFront = true;
    }

    /* Requests the sytem to intake a cube from the double substation */
    public void requestIntakeSingleSubstationCone() {
        unsetAllRequests();
        requestSingleSubstationCone = true;
    }

    /* Requests the system to pre score */
    public void requestPreScore(Level level, GamePiece piece, boolean goSlow) {
        unsetAllRequests();
        requestPreScore = true;
        this.level = level;
        this.piece = piece;
        this.goSlow = goSlow;
    }

       /* Overload which requests the system to pre score and defaults to moving fast */
    public void requestPreScore(Level level, GamePiece piece) {
        requestPreScore(level, piece, false);
    }

    /* Requests the system to score */
    public void requestScore() {
        unsetAllRequests();
        requestScore = true;
    }

    /* Requests the system to floor intake a cone */
    public void requestFloorIntakeCone() {
        unsetAllRequests();
        requestFloorIntakeCone = true;
    }

    /* Requests the system to throw */
    public void requestThrow() {
        unsetAllRequests();
        throwSetpoint = ELEVATOR_THROW_POSE + Units.inchesToMeters(Math.random() * 2.0 - 1.0);
        requestThrow = true;
    }
    
    /* Sets all of the requests to false */
    private void unsetAllRequests() {
        requestFloorIntakeCube = false;
        requestSingleSubstationCone = false;
        requestHPIntakeCone = false;
        requestPreScore = false;
        requestScore = false;
        requestSpit = false;
        requestFloorIntakeCone = false;
        requestThrow = false;
        requestSpitCubeFront = false;
        goSlow = false;
    }

    /** Zeroes all sensors */
    public void zeroSensors() {
        elevatorArmLowLevel.zeroSensors();
        floorIntake.zeroSensors();
    }

    /** Returns the system state */
    public SuperstructureState getSystemState() {
        return systemState;
    }

    /** Returns whether or not the elevator is at a certain height */
    public boolean atElevatorSetpoint(double height) {
        return elevatorArmLowLevel.atElevatorSetpoint(height);
    }

    /** Returns whether or not the arm is at a certain height */
    public boolean atArmSetpoint(double angle) {
        return elevatorArmLowLevel.atArmSetpoint(angle);
    }
    
    /** Returns the height of the elevator */
    public double getElevatorHeight() {
        return elevatorArmLowLevel.elevatorInputs.posMeters;
    }

    /** Returns the angle of the arm */
    public double getArmAngle() {
        return elevatorArmLowLevel.armInputs.angleDegrees;
    }

    /** Returns whether or not the superstructure has homed once */
    public boolean homedOnce() {
        return homedOnce;
    }

    /** Returns whether or not the system is holding a gampiece in idle */
    public boolean hasGampiece() {
        return systemState == SuperstructureState.IDLE && (Math.abs(endEffector.getMotorRPM()) < 10.0 || endEffector.getBeamBreakTriggered()) && DriverStation.isEnabled() == true;
    }

}


package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.commons.BreadUtil;
import frc.robot.commons.TunableNumber;
import frc.robot.subsystems.elevatorarm.ArmIO;
import frc.robot.subsystems.elevatorarm.ElevatorArmLowLevel;
import frc.robot.subsystems.elevatorarm.ElevatorIO;
import frc.robot.subsystems.elevatorarm.ElevatorArmLowLevel.ElevatorArmSystemStates;
import frc.robot.subsystems.endeffector.EndEffectorIO;
import frc.robot.subsystems.floorintake.FloorIntake;
import frc.robot.subsystems.floorintake.FloorIntakeIO;
import frc.robot.subsystems.floorintake.FloorIntake.FloorIntakeStates;

import static frc.robot.Constants.Elevator.*;

import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Arm.*;

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
    Timer currentTriggerTimer = new Timer();
    boolean currentTriggerTimerStarted = false;


    private Level level = Level.LOW;
    private GamePiece piece = GamePiece.CUBE;

    /* Presets */
    TunableNumber preCubeHighHeight = new TunableNumber("Elevator/PreCubeHighHeight", ELEVATOR_PRE_CUBE_HIGH);
    TunableNumber preConeHighHeight = new TunableNumber("Elevator/PreConeHighHeight", ELEVATOR_PRE_CONE_HIGH);
    TunableNumber coneSlamHeight = new TunableNumber("Elevator/ConeSlamHighHeight", ELEVATOR_CONE_SLAM_HIGH);
    TunableNumber conePulloutHeight = new TunableNumber("Elevator/ConePulloutHighHeight", ELEVATOR_CONE_PULL_OUT_HIGH);
    TunableNumber cubeOffset = new TunableNumber("Elevator/CubeOffsetHeight", ELEVATOR_CUBE_OFFSET);
    TunableNumber coneOffset = new TunableNumber("Elevator/ConeOffsetHeight", ELEVATOR_CONE_OFFSET);
    TunableNumber preLowHeight = new TunableNumber("Elevator/PreLowHeight", ELEVATOR_PRE_LOW);
    TunableNumber floorIntakeCubeHeight = new TunableNumber("Elevator/FloorIntakeCubeHeight", ELEVATOR_FLOOR_INTAKE_CUBE);
    TunableNumber doubleSubstationConeHeight = new TunableNumber("Elevator/DoubleSubstationConeHeight", ELEVATOR_DOUBLE_SUBSTATION_CONE);
    TunableNumber doubleSubstationCubeHeight = new TunableNumber("Elevator/DoubleSubstationCubeHeight", ELEVATOR_DOUBLE_SUBSTATION_CUBE);

    TunableNumber cubeAngle = new TunableNumber("Arm/ArmCubeAngle", ARM_PRE_SCORE_CUBE);
    TunableNumber coneAngle = new TunableNumber("Arm/ArmConeAngle", ARM_PRE_SCORE_CONE);
    TunableNumber slamConeAngle = new TunableNumber("Arm/ArmSlamConeAngle", ARM_SLAM_CONE);
    TunableNumber lowAngle = new TunableNumber("Arm/ArmLowAngle", ARM_PRE_SCORE_LOW);
    TunableNumber floorIntakeCubeAngle = new TunableNumber("Arm/ArmFloorIntakeCube", ARM_FLOOR_INTAKE_CUBE);
    TunableNumber doubleSubstationConeAngle = new TunableNumber("Arm/ArmDoubleSubstationConeAngle", ARM_DOUBLE_SUBSTATION_CONE);
    TunableNumber doubleSubstationCubeAngle = new TunableNumber("Arm/ArmDoubleSubstationCubeAngle", ARM_DOUBLE_SUBSTATION_CUBE);
    // TunableNumber floorIntakeOutput = new TunableNumber("FloorIntake/Output", 0.0);

    /* System States */
    public enum SuperstructureState {
        PRE_HOME,
        HOMING_ELEVATOR_ARM,
        HOMING_INTAKE, 
        IDLE,
        FLOOR_INTAKE_CUBE,
        HP_INTAKE_CUBE,
        HP_INTAKE_CONE,
        SPIT,
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

        /* On loops */
        elevatorArmLowLevel.onLoop();
        endEffector.onLoop();
        floorIntake.onLoop();

        /* Statemachinet things */
        SuperstructureState nextSystemState = systemState;
        if (systemState == SuperstructureState.PRE_HOME) {
            // Outputs
            endEffector.idling();

            // Transitions
            if (requestHome) {
                elevatorArmLowLevel.requestHome();
                nextSystemState = SuperstructureState.HOMING_ELEVATOR_ARM;
            }
        } else if (systemState == SuperstructureState.HOMING_ELEVATOR_ARM) {
            // Outputs
            endEffector.idling();

            // Transitions 
            if (elevatorArmLowLevel.getSystemState() == ElevatorArmSystemStates.IDLE) {
                floorIntake.requestHome();
                nextSystemState = SuperstructureState.HOMING_INTAKE;
            }
        } else if (systemState == SuperstructureState.HOMING_INTAKE) {
            // Outputs
            endEffector.idling();
            elevatorArmLowLevel.requestDesiredState(0.1, 90.0);


            // Transitions 
            if (floorIntake.getSystemState() == FloorIntakeStates.IDLE) {
                nextSystemState = SuperstructureState.IDLE;
                requestHome = false;
            }
        } else if (systemState == SuperstructureState.IDLE) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(0.25, 90.0);
            endEffector.holdGamePiece();
            floorIntake.requestClosedLoop(0.0, 50.0);
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
                nextSystemState = SuperstructureState.SPIT;
            }else if (requestPreScore && level == Level.LOW) {
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
            floorIntake.requestClosedLoop(0.7, 148.0);

            if (endEffector.getStatorCurrent() > 14.0 && !currentTriggerTimerStarted) {
                currentTriggerTimerStarted = true;
                currentTriggerTimer.reset();
                currentTriggerTimer.start();
            } 

            if (endEffector.getStatorCurrent() < 14.0) {
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
            floorIntake.requestClosedLoop(0.0, 50.0);

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (!requestHPIntakeCone) {
                nextSystemState = SuperstructureState.IDLE;
            } else if (endEffector.getStatorCurrent() > 13.0) {
                requestHPIntakeCone = false;
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.HP_INTAKE_CUBE) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(doubleSubstationCubeHeight.get(), doubleSubstationCubeAngle.get());
            endEffector.intakeCube();
            floorIntake.requestClosedLoop(0.0, 50.0);

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (!requestHPIntakeCube) {
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.SPIT) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(0.1, 90.0);
            endEffector.spitCube();
            floorIntake.requestClosedLoop(-0.5, 50.0);

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (!requestSpit) {
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.PRE_PLACE_PIECE_LOW) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(preLowHeight.get(), lowAngle.get());
            endEffector.holdGamePiece();
            floorIntake.requestClosedLoop(0.0, 50.0);

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
            floorIntake.requestClosedLoop(0.0, 50.0);

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
            endEffector.holdGamePiece();
            floorIntake.requestClosedLoop(0.0, 50.0);

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
            floorIntake.requestClosedLoop(0.0, 50.0);

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
            floorIntake.requestClosedLoop(0.0, 50.0);

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
            endEffector.idling();
            floorIntake.requestClosedLoop(0.0, 50.0);

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
            floorIntake.requestClosedLoop(0.0, 50.0);

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (!requestScore) {
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.FLOOR_INTAKE_CONE_A) {
            // Outputs
            endEffector.idling();
            if (floorIntake.getRollerCurrent() > 47.0) {
                floorIntake.requestClosedLoop(-0.75, 15.0);
            } else {
                floorIntake.requestClosedLoop(-0.75, 157.0);
            }
            elevatorArmLowLevel.requestDesiredState(0.1, 90.0);

            // Transitions
            if (!requestFloorIntakeCone) {
                nextSystemState = SuperstructureState.IDLE;
            } else if (floorIntake.getAngle() < 120.0 && floorIntake.getRollerCurrent() > 47.0) {
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
            floorIntake.requestClosedLoop(-0.75, 15.0);
            endEffector.idling();

            // Transitions
            if (elevatorArmLowLevel.atArmSetpoint(2.0) && elevatorArmLowLevel.atElevatorSetpoint(0.5)) {
                nextSystemState = SuperstructureState.FLOOR_INTAKE_CONE_C;
            } else if (!requestFloorIntakeCone) {
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.FLOOR_INTAKE_CONE_C) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(0.25, 2.0);
            if (elevatorArmLowLevel.atElevatorSetpoint(0.25)) {
                floorIntake.requestClosedLoop(0.25, 15.0);
            } else {
                floorIntake.requestClosedLoop(-0.75, 15.0);
            }
            endEffector.intakeCone();

            if (endEffector.getStatorCurrent() > 28.0 && !currentTriggerTimerStarted) {
                currentTriggerTimerStarted = true;
                currentTriggerTimer.reset();
                currentTriggerTimer.start();
            } 

            if (endEffector.getStatorCurrent() < 28.0) {
                currentTriggerTimer.reset();
                currentTriggerTimer.stop();
                currentTriggerTimerStarted = false;
            }

            // Transitions
            if (!requestFloorIntakeCone) {
                nextSystemState = SuperstructureState.IDLE;
            } else if (currentTriggerTimer.get() > 0.5) {
                nextSystemState = SuperstructureState.IDLE;
                requestFloorIntakeCone = false;
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
    public void requestFloorIntakeCube() {
        unsetAllRequests();
        requestFloorIntakeCube = true;
    }

    /* Requests the system to spit */
    public void requestSpit() {
        unsetAllRequests();
        requestSpit = true;
    }

    /* Requests the sytem to intake a cone from the double substation */
    public void requestIntakeConeDoubleSubstation() {
        unsetAllRequests();
        requestHPIntakeCone = true;
    }

    /* Requests the sytem to intake a cube from the double substation */
    public void requestIntakeCubeDoubleSubstation() {
        unsetAllRequests();
        requestHPIntakeCube = true;
    }

    /* Requests the system to pre score */
    public void requestPreScore(Level level, GamePiece piece) {
        unsetAllRequests();
        requestPreScore = true;
        this.level = level;
        this.piece = piece;
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
    
    /* Sets all of the requests to false */
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

}


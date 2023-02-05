package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
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

import static frc.robot.Constants.Elevator.*;

import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Arm.*;

public class Superstructure extends SubsystemBase {

    /* IO and subsystems */
    private ElevatorArmLowLevel elevatorArmLowLevel;
    private EndEffector endEffector;

    /* State variables */
    private double mStateStartTime = 0.0;
    private SuperstructureState systemState = SuperstructureState.PRE_HOME;
    private boolean requestHome = false;
    private boolean requestFloorIntakeCube = false;
    private boolean requestHPIntakeCube = false;
    private boolean requestHPIntakeCone = false;
    private boolean requestPreScore = false;
    private boolean requestScore = false;

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

    /* System States */
    public enum SuperstructureState {
        PRE_HOME,
        HOMING,
        IDLE,
        FLOOR_INTAKE_CUBE,
        HP_INTAKE_CUBE,
        HP_INTAKE_CONE,
        PRE_PLACE_PIECE_LOW,
        PRE_PLACE_CUBE,
        PRE_PLACE_CONE,
        EXHAUSTING_PIECE_LOW,
        EXHAUSTING_CUBE,
        SLAM_CONE, 
        PULL_OUT_CONE
    }

    public enum Level {
        LOW, MID, HIGH
    }

    public enum GamePiece {
        CONE, CUBE
    }

    /* Set IO and subsystems to what they should be equal to  */
    public Superstructure(ElevatorIO elevatorIO, ArmIO armIO, EndEffectorIO endEffectorIO) {
        elevatorArmLowLevel = new ElevatorArmLowLevel(armIO, elevatorIO);
        endEffector = new EndEffector(endEffectorIO);
    }

    @Override
    public void periodic() {

        /* Logs */
        Logger.getInstance().recordOutput("SuperstructureState", systemState.toString());

        /* On loops */
        elevatorArmLowLevel.onLoop();
        endEffector.onLoop();

        /* Statemachinet things */
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

            // Transitions 
            if (elevatorArmLowLevel.getSystemState() == ElevatorArmSystemStates.IDLE) {
                nextSystemState = SuperstructureState.IDLE;
                requestHome = false;
            }
        } else if (systemState == SuperstructureState.IDLE) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(0.025, 90.0);
            endEffector.holdGamePiece();

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (requestFloorIntakeCube) {
                nextSystemState = SuperstructureState.FLOOR_INTAKE_CUBE;
            } else if (requestHPIntakeCone) {
                nextSystemState = SuperstructureState.HP_INTAKE_CONE;
            } else if (requestHPIntakeCube) {
                nextSystemState = SuperstructureState.HP_INTAKE_CUBE;
            } else if (requestPreScore && level == Level.LOW) {
                nextSystemState = SuperstructureState.PRE_PLACE_PIECE_LOW;
            } else if (requestPreScore && piece == GamePiece.CUBE) {
                nextSystemState = SuperstructureState.PRE_PLACE_CUBE;
            } else if (requestPreScore && piece == GamePiece.CONE) {
                nextSystemState = SuperstructureState.PRE_PLACE_CONE;
            }
        } else if (systemState == SuperstructureState.FLOOR_INTAKE_CUBE) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(floorIntakeCubeHeight.get(), floorIntakeCubeAngle.get());
            endEffector.intakeCube();

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (!requestFloorIntakeCube) {
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.HP_INTAKE_CONE) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(doubleSubstationConeHeight.get(), doubleSubstationConeAngle.get());
            endEffector.intakeCone();

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (!requestHPIntakeCone) {
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.HP_INTAKE_CUBE) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(doubleSubstationCubeHeight.get(), doubleSubstationCubeAngle.get());
            endEffector.intakeCube();

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (!requestHPIntakeCube) {
                nextSystemState = SuperstructureState.IDLE;
            }
        } else if (systemState == SuperstructureState.PRE_PLACE_PIECE_LOW) {
            // Outputs
            elevatorArmLowLevel.requestDesiredState(preLowHeight.get(), lowAngle.get());
            endEffector.holdGamePiece();

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

            // Transitions
            if (requestHome) {
                nextSystemState = SuperstructureState.PRE_HOME;
            } else if (!requestScore) {
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
    } 

    /* Requests the entire system to go into its idling mode */
    public void requestIdle() {
        unsetAllRequests();
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
    
    /* Sets all of the requests to false */
    private void unsetAllRequests() {
        requestFloorIntakeCube = false;
        requestHPIntakeCube = false;
        requestHPIntakeCone = false;
        requestPreScore = false;
        requestScore = false;
    }

    /** Zeroes all sensors */
    public void zeroSensors() {
        elevatorArmLowLevel.zeroSensors();
    }

}


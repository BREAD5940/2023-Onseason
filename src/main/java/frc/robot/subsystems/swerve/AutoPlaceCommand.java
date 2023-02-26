package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.AllianceFlipUtil;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.SuperstructureState;

import static frc.robot.FieldConstants.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Elevator.*;

public class AutoPlaceCommand extends CommandBase {

    private final PIDController zoneOneXController = new PIDController(4.0, 0.0, 0.0);
    private final PIDController zoneOneYController = new PIDController(4.0, 0.0, 0.0);
    private final PIDController zoneOneThetaController = new PIDController(5.0, 0.0, 0.0);

    private Pose2d goal;
    private int nodeNumber;
    private Level level;
    private final Swerve swerve;
    private final Superstructure superstructure;
    private boolean requestedPreScore = false;
    private boolean requestedScore = false;

    public AutoPlaceCommand(Swerve swerve, Superstructure superstructure) {
        zoneOneThetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.swerve = swerve;
        this.superstructure = superstructure;
        addRequirements(swerve, superstructure);
    }

    @Override
    public void initialize() {
        level = Robot.scoringLevel;
        nodeNumber = 9 - (Robot.scoringSquare * 3 + Robot.scoringSpot);
        if (nodeNumber == 2 || nodeNumber == 5 || nodeNumber == 8) {
            goal = new Pose2d(
                1.95,
                Grids.lowTranslations[nodeNumber - 1].getY(),
                new Rotation2d(Math.PI)
            );
        } else {
            goal = new Pose2d(
                1.83,
                Grids.lowTranslations[nodeNumber - 1].getY(),
                new Rotation2d(Math.PI)
            );
        }
        goal = AllianceFlipUtil.apply(goal);
        requestedPreScore = false;
        requestedScore = false;
    }

    @Override
    public void execute() {
        Pose2d realGoal = goal;
        Pose2d poseError = realGoal.relativeTo(RobotContainer.poseEstimator.getLatestPose());
        if (Math.abs(poseError.getY()) > Units.inchesToMeters(3.0)) {
            realGoal = new Pose2d(Robot.alliance == Alliance.Red ? fieldLength - 2.3 : 2.3, goal.getY(), goal.getRotation());
        }
        double xFeedback = zoneOneXController.calculate(RobotContainer.poseEstimator.getLatestPose().getX(), realGoal.getX());
        double yFeedback = zoneOneYController.calculate(RobotContainer.poseEstimator.getLatestPose().getY(), realGoal.getY());
        double thetaFeedback = zoneOneThetaController.calculate(RobotContainer.poseEstimator.getLatestPose().getRotation().getRadians(), realGoal.getRotation().getRadians());

        xFeedback = MathUtil.clamp(xFeedback, -1, 1);
        yFeedback = MathUtil.clamp(yFeedback, -1, 1);
        thetaFeedback = MathUtil.clamp(thetaFeedback, -1.0, 1.0);
        Logger.getInstance().recordOutput("AutoPlaceGoal", realGoal);

        swerve.requestVelocity(new ChassisSpeeds(xFeedback, yFeedback, thetaFeedback), true);

        if (Math.abs(poseError.getX()) < Units.inchesToMeters(1.5) && 
        Math.abs(poseError.getY()) < Units.inchesToMeters(1.0) && 
        Math.abs(poseError.getRotation().getDegrees()) < 1.0 &&
        !requestedPreScore) {
            requestedPreScore = true;
            if (nodeNumber == 2 || nodeNumber == 5 || nodeNumber == 8) {
                superstructure.requestPreScore(level, GamePiece.CUBE);
            } else {
                superstructure.requestPreScore(level, GamePiece.CONE);
            }
        }

        if (superstructure.getSystemState() == SuperstructureState.PRE_PLACE_CONE) {
            if (level == Level.MID && superstructure.atElevatorSetpoint(ELEVATOR_PRE_CONE_HIGH - ELEVATOR_CONE_OFFSET)) {
                requestedScore = true;
                superstructure.requestScore();
            } else if (level == Level.HIGH && superstructure.atElevatorSetpoint(ELEVATOR_PRE_CONE_HIGH)) {
                requestedScore = true;
                superstructure.requestScore();
            }
        } else if (superstructure.getSystemState() == SuperstructureState.PRE_PLACE_CUBE) {
            if (level == Level.MID && superstructure.atElevatorSetpoint(ELEVATOR_PRE_CUBE_HIGH - ELEVATOR_CUBE_OFFSET)) {
                requestedScore = true;
                superstructure.requestScore();
            } else if (level == Level.HIGH && superstructure.atElevatorSetpoint(ELEVATOR_PRE_CUBE_HIGH)) {
                requestedScore = true;
                superstructure.requestScore();
            }
        } else if (superstructure.getSystemState() == SuperstructureState.PRE_PLACE_PIECE_LOW) {
            if (superstructure.atElevatorSetpoint(ELEVATOR_PRE_LOW)) {
                requestedScore = true;
                superstructure.requestScore();
            }
        }
    }
    
}

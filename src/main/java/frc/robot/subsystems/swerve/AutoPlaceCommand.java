package frc.robot.subsystems.swerve;

import static frc.robot.Constants.Vision.HIGH_TAPE_OFF_GROUND;
import static frc.robot.Constants.Vision.MID_TAPE_OFF_GROUND;
import static frc.robot.Constants.Vision.X_SCORING_POSITION;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FieldConstants.Grids;
import frc.robot.RobotContainer;
import frc.robot.commons.AllianceFlipUtil;
import frc.robot.commons.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.commons.LoggedTunableNumber;
import frc.robot.commons.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.vision.limelight.LimelightDetectionsClassifier;

public class AutoPlaceCommand extends CommandBase {

    private Pose3d nodeLocation;
    private Pose2d targetRobotPose;
    private Pose2d preTargetRobotPose;
    private boolean shouldUseLimelight = false;
    private boolean isUsingLimelight = false;
    private boolean isCubeNode = false;
    private boolean linedUp = false;
    private boolean scored = false;
    private boolean converged = false;
    private Level level;
    private GamePiece gamePiece;

    private Supplier<GamePiece> gamePieceSup; 
    private Supplier<Level> levelSup;

    private final PIDController xController = new PIDController(4.0, 0.0, 0.004);
    private final PIDController yController = new PIDController(4.0, 0.0, 0.004);
    private final PIDController thetaController = new PIDController(5.0, 0.0, 0.0);

    private final Swerve swerve;
    private final Superstructure superstructure;

    LoggedTunableNumber thetaP = new LoggedTunableNumber("AutoPlace/ThetaP", 5.0);

    public AutoPlaceCommand(Swerve swerve, Superstructure superstructure, Supplier<GamePiece> gamePieceSupplier, Supplier<Level> levelSup) {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.swerve = swerve;
        this.superstructure = superstructure;
        this.gamePieceSup = gamePieceSupplier;
        this.levelSup = levelSup;
        addRequirements(swerve, superstructure);
    }

    @Override   
    public void initialize() {
        // Read selection from the operator controls
        isUsingLimelight = false;
        linedUp = false;
        scored = false;
        converged = false;
        this.level = levelSup.get();
        this.gamePiece = gamePieceSup.get();

        int scoringLocation = getScoringLocationClosestToRobot(level, gamePiece); //scoringLocationSup.get();

        // if (DriverStation.getAlliance() == Alliance.Blue) {
        //     scoringLocation = 10 - scoringLocation;
        // }
        // if (level == Level.HIGH) {
        //     Translation2d xyNodeTranslation = AllianceFlipUtil.apply(Grids.highTranslations[scoringLocation - 1]);
        //     nodeLocation = new Pose3d(
        //             xyNodeTranslation.getX(),
        //             xyNodeTranslation.getY(),
        //             HIGH_TAPE_OFF_GROUND,
        //             new Rotation3d());
        // } else if (level == Level.MID) {
        //     Translation2d xyNodeTranslation = AllianceFlipUtil.apply(Grids.midTranslations[scoringLocation - 1]);
        //     nodeLocation = new Pose3d(
        //             xyNodeTranslation.getX(),
        //             xyNodeTranslation.getY(),
        //             MID_TAPE_OFF_GROUND,
        //             new Rotation3d());
        // } else if (level == Level.LOW) {
        //     Translation2d xyNodeTranslation = AllianceFlipUtil.apply(Grids.lowTranslations[scoringLocation - 1]);
        //     nodeLocation = new Pose3d(
        //             xyNodeTranslation.getX(),
        //             xyNodeTranslation.getY(),
        //             0.0,
        //             new Rotation3d());
        // }
        isCubeNode = (scoringLocation == 2 || scoringLocation == 5 || scoringLocation == 8);
        if (isCubeNode) {
            targetRobotPose = new Pose2d(X_SCORING_POSITION + 0.1, nodeLocation.getY(), new Rotation2d(Math.PI));
        } else {
            if (level == Level.HIGH) {
                targetRobotPose = new Pose2d(X_SCORING_POSITION - 0.04, nodeLocation.getY(), new Rotation2d(Math.PI));
            } else {
                targetRobotPose = new Pose2d(X_SCORING_POSITION - 0.03, nodeLocation.getY(), new Rotation2d(Math.PI));
            }
        }
        preTargetRobotPose = new Pose2d(targetRobotPose.getX() + Units.inchesToMeters(4), targetRobotPose.getY(), targetRobotPose.getRotation());
        shouldUseLimelight = !(scoringLocation == 2 || scoringLocation == 5 || scoringLocation == 8 || level == Level.LOW);

        Logger.getInstance().recordOutput("AutoPlace/TargetRobotPose", targetRobotPose);
    }

    
    @Override
    public void execute() { 
        thetaController.setP(thetaP.get());

        Pose2d realGoal = AllianceFlipUtil.apply(targetRobotPose);
        Pose2d poseError = realGoal.relativeTo(RobotContainer.poseEstimator.getLatestPose());
        if (Math.abs(poseError.getY()) > Units.inchesToMeters(6.0)) {
            realGoal = AllianceFlipUtil.apply(preTargetRobotPose);
        }

        Pose2d measurement = RobotContainer.poseEstimator.getLatestPose();
        if (shouldUseLimelight) {
            if (poseError.getTranslation().getNorm() < Units.inchesToMeters(12.0) || isUsingLimelight) { // Start using limelight
                List<TimestampedVisionUpdate> limelightUpdate = new ArrayList<>();
                
                ArrayList<Pose3d> poses = RobotContainer.limelightVision.getTargets(level == Level.HIGH ? true : false);

                int indexOfPoseClosestToTarget = getIndexOfPoseClosestToTarget(poses);

                // isUsingLimelight = true;

                Logger.getInstance().recordOutput("AutoPlace/IndexOfPoseClosestToTarget", indexOfPoseClosestToTarget);
                if (indexOfPoseClosestToTarget != -1) {
                    LimelightTarget_Retro detection = RobotContainer.limelightVision.getRawDetections()[indexOfPoseClosestToTarget];
                    Pose3d llRobotPose = LimelightDetectionsClassifier.targetPoseToRobotPose(nodeLocation.getTranslation(), new Pose3d(RobotContainer.poseEstimator.getLatestPose()), RobotContainer.limelightVision.getRawDetections()[indexOfPoseClosestToTarget]);

                    Pose2d convergenceError = llRobotPose.toPose2d().relativeTo(RobotContainer.poseEstimator.getLatestPose());
                    if (convergenceError.getTranslation().getNorm() < Units.inchesToMeters(1.0) && Math.abs(convergenceError.getRotation().getDegrees()) < 1.0) {
                        converged = true;
                    } else {
                        converged = false;
                    }

                    limelightUpdate.add(new TimestampedVisionUpdate(
                        RobotContainer.limelightVision.getAssociatedTimestamp(), 
                        llRobotPose.toPose2d(), 
                        VecBuilder.fill(0.00005, 0.00005, 1.0E6)
                    ));

                    Logger.getInstance().recordOutput("LimelightEstimatedRobotPose", llRobotPose);

                    
                    RobotContainer.poseEstimator.addVisionData(limelightUpdate);
                } else {
                    converged = false;
                }
            }
        }

        
        double xFeedback = xController.calculate(measurement.getX(), realGoal.getX());
        double yFeedback = yController.calculate(measurement.getY(), realGoal.getY());
        double thetaFeedback = thetaController.calculate(
            measurement.getRotation().getRadians(),
            realGoal.getRotation().getRadians());

        xFeedback = MathUtil.clamp(xFeedback, -1, 1);
        yFeedback = MathUtil.clamp(yFeedback, -1, 1);
        thetaFeedback = MathUtil.clamp(thetaFeedback, -1.0, 1.0);

        if (shouldUseLimelight) {
            if (poseError.getTranslation().getNorm() < Units.inchesToMeters(1.0) && Math.abs(poseError.getRotation().getDegrees()) < 1.0 && !linedUp && converged) {
                linedUp = true;
                superstructure.requestPreScore(level, isCubeNode ? GamePiece.CUBE : GamePiece.CONE);
            } 
        } else {
            if (poseError.getTranslation().getNorm() < Units.inchesToMeters(3.0) && Math.abs(poseError.getRotation().getDegrees()) < 2.0 && !linedUp) {
                linedUp = true;
                superstructure.requestPreScore(level, isCubeNode ? GamePiece.CUBE : GamePiece.CONE);
            } 
        }

        if (!scored) {
            if (level == Level.LOW) {
                if (superstructure.atElevatorSetpoint(Superstructure.preLowHeight.get())) {
                    superstructure.requestScore();
                    scored = true;
                }
            } else if (isCubeNode) {
                if (superstructure.atElevatorSetpoint(level == Level.HIGH ? Superstructure.preCubeHighHeight.get() : Superstructure.preCubeHighHeight.get() - Superstructure.cubeOffset.get())) {
                    superstructure.requestScore();
                    scored = true;
                }
            } else if (!isCubeNode) {
                if (level == Level.HIGH) {
                    if (superstructure.getElevatorHeight() > Superstructure.preConeHighHeight.get() - 0.5) {
                        superstructure.requestScore();
                        scored = true;
                    } 
                } else if (level == Level.MID) {
                    if (superstructure.getElevatorHeight() > Superstructure.preConeHighHeight.get() - Superstructure.coneOffset.get() - 0.5 && RobotContainer.superstructure.getSystemState() == SuperstructureState.PRE_PLACE_CONE) {
                        superstructure.requestScore();
                        scored = true;
                    }
                }
            }
        }
        
        if (!linedUp) {
            swerve.requestVelocity(new ChassisSpeeds(xFeedback, yFeedback, thetaFeedback), true, false);
        } else {
            swerve.requestVelocity(new ChassisSpeeds(0, 0, 0), true, false);
        }

        Logger.getInstance().recordOutput("AutoPlace/RealGoal", realGoal);
        Logger.getInstance().recordOutput("AutoPlace/TargetRobotPose", targetRobotPose);
        Logger.getInstance().recordOutput("AutoPlace/PreTargetRobotPose", preTargetRobotPose);
        Logger.getInstance().recordOutput("AutoPlace/Converged", converged);
        Logger.getInstance().recordOutput("Autoplace/PoseError", poseError);
        
    }

    private int getIndexOfPoseClosestToTarget(ArrayList<Pose3d> poses) {
        if (poses.size() > 0) {
            int index = 0;
            for (int i = 1; i < poses.size(); i++) {
                double error = poses.get(i).relativeTo(nodeLocation).getTranslation().getNorm();
                double otherError = poses.get(index).relativeTo(nodeLocation).getTranslation().getNorm();
                if (error < otherError) {
                    index = i;
                }
            }
            double lowestError = poses.get(index).relativeTo(nodeLocation).getTranslation().getNorm();
            if (lowestError < Units.inchesToMeters(8.0)) {
                return index;
            }
        }
        return -1;
    }

    private int getScoringLocationClosestToRobot(Level level, GamePiece gamePiece) {
        Pose3d closestNode = new Pose3d();
        Pose3d testNode = new Pose3d();
        int closestScoringLocation = -1;
        double errorOfCurrentClosest = Double.POSITIVE_INFINITY;
        Pose2d currentPose = RobotContainer.poseEstimator.getLatestPose();
        int scoringLocation = 0;
        for (int i = 1; i <= 9; i++) {
            scoringLocation = i;
            if (DriverStation.getAlliance() == Alliance.Blue) {
                scoringLocation = 10 - i;
            }

            boolean isCubeNode = (scoringLocation == 2 || scoringLocation == 5 || scoringLocation == 8);

            if (level == Level.HIGH) {
                Translation2d xyNodeTranslation = AllianceFlipUtil.apply(Grids.highTranslations[scoringLocation - 1]);
                testNode = new Pose3d(
                        xyNodeTranslation.getX(),
                        xyNodeTranslation.getY(),
                        HIGH_TAPE_OFF_GROUND,
                        new Rotation3d());
            } else if (level == Level.MID) {
                Translation2d xyNodeTranslation = AllianceFlipUtil.apply(Grids.midTranslations[scoringLocation - 1]);
                testNode = new Pose3d(
                        xyNodeTranslation.getX(),
                        xyNodeTranslation.getY(),
                        MID_TAPE_OFF_GROUND,
                        new Rotation3d());
            } else if (level == Level.LOW) {
                Translation2d xyNodeTranslation = AllianceFlipUtil.apply(Grids.lowTranslations[scoringLocation - 1]);
                testNode = new Pose3d(
                        xyNodeTranslation.getX(),
                        xyNodeTranslation.getY(),
                        0.0,
                        new Rotation3d());
            }
            Pose2d testRobotPose;
            if (isCubeNode) {
                testRobotPose = new Pose2d(X_SCORING_POSITION + 0.1, testNode.getY(), new Rotation2d(Math.PI));
            } else {
                if (level == Level.HIGH) {
                    testRobotPose = new Pose2d(X_SCORING_POSITION - 0.01, testNode.getY(), new Rotation2d(Math.PI));
                } else {
                    testRobotPose = new Pose2d(X_SCORING_POSITION - 0.03, testNode.getY(), new Rotation2d(Math.PI));
                }
            }
            testRobotPose = AllianceFlipUtil.apply(testRobotPose);
            double error = testRobotPose.relativeTo(currentPose).getTranslation().getNorm();
            
            if (error < errorOfCurrentClosest) {
                if ((gamePiece == GamePiece.CUBE && isCubeNode) || (gamePiece == GamePiece.CONE && !isCubeNode)) {
                    errorOfCurrentClosest = error;
                    closestScoringLocation = scoringLocation;
                    closestNode = testNode;
                }
            }
        }
        nodeLocation = closestNode;
        return closestScoringLocation;
    }

}
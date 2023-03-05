package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commons.AllianceFlipUtil;
import frc.robot.commons.LoggedTunableNumber;
import frc.robot.commons.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.commons.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.vision.limelight.LimelightDetectionsClassifier;

import static frc.robot.FieldConstants.*;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Vision.*;
import static frc.robot.Constants.Elevator.*;

public class AutoPlaceCommand extends CommandBase {

    private Pose3d nodeLocation;
    private Pose2d targetRobotPose;
    private Pose2d preTargetRobotPose;
    private boolean shouldUseLimelight = false;
    private boolean isUsingLimelight = false;
    private boolean isCubeNode = false;
    private boolean linedUp = false;
    private boolean scored = false;
    private Level level;

    private final PIDController xController = new PIDController(4.0, 0.0, 0.004);
    private final PIDController yController = new PIDController(4.0, 0.0, 0.004);
    private final PIDController thetaController = new PIDController(5.0, 0.0, 0.0);

    private final Swerve swerve;
    private final Superstructure superstructure;

    public AutoPlaceCommand(Swerve swerve, Superstructure superstructure) {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.swerve = swerve;
        this.superstructure = superstructure;
        addRequirements(swerve, superstructure);
    }

    @Override   
    public void initialize() {
        // Read selection from the operator controls
        isUsingLimelight = false;
        linedUp = false;
        scored = false;
        int scoringLocation = RobotContainer.operatorControls.getLastSelectedScoringLocation();
        if (DriverStation.getAlliance() == Alliance.Blue) {
            scoringLocation = 10 - scoringLocation;
        }
        this.level = RobotContainer.operatorControls.getLastSelectedLevel();
        // if (level == Level.HIGH) {
        //     Translation2d xyNodeTranslation = AllianceFlipUtil.apply(Grids.highTranslations[scoringLocation - 1]);
        //     nodeLocation = new Pose3d(
        //             xyNodeTranslation.getX(),
        //             xyNodeTranslation.getY(),
        //             HIGH_TAPE_OFF_GROUND,
        //             new Rotation3d());
        // } else 
        if (level == Level.HIGH || level == Level.MID) {
            Translation2d xyNodeTranslation = AllianceFlipUtil.apply(Grids.midTranslations[scoringLocation - 1]);
            nodeLocation = new Pose3d(
                    xyNodeTranslation.getX(),
                    xyNodeTranslation.getY(),
                    MID_TAPE_OFF_GROUND,
                    new Rotation3d());
        } else {
            Translation2d xyNodeTranslation = AllianceFlipUtil.apply(Grids.lowTranslations[scoringLocation - 1]);
            nodeLocation = new Pose3d(
                    xyNodeTranslation.getX(),
                    xyNodeTranslation.getY(),
                    0.0,
                    new Rotation3d());
        }
        targetRobotPose = new Pose2d(X_SCORING_POSITION, nodeLocation.getY(), new Rotation2d(Math.PI));
        preTargetRobotPose = new Pose2d(targetRobotPose.getX() + 0.5, targetRobotPose.getY(), targetRobotPose.getRotation());
        shouldUseLimelight = !(scoringLocation == 2 || scoringLocation == 5 || scoringLocation == 8 || level == Level.LOW);
        isCubeNode = (scoringLocation == 2 || scoringLocation == 5 || scoringLocation == 8);
    }

    @Override
    public void execute() { 
        Pose2d realGoal = AllianceFlipUtil.apply(targetRobotPose);
        Pose2d poseError = realGoal.relativeTo(RobotContainer.poseEstimator.getLatestPose());
        if (Math.abs(poseError.getY()) > Units.inchesToMeters(6.0)) {
            realGoal = AllianceFlipUtil.apply(preTargetRobotPose);
        }

        Pose2d measurement = RobotContainer.poseEstimator.getLatestPose();
        if (shouldUseLimelight) {
            if (poseError.getTranslation().getNorm() < Units.inchesToMeters(12.0) || isUsingLimelight) { // Start using limelight
                List<TimestampedVisionUpdate> limelightUpdate = new ArrayList<>();
                
                ArrayList<Pose3d> poses = RobotContainer.limelightVision.getTargets(false);

                int indexOfPoseClosestToTarget = getIndexOfPoseClosestToTarget(poses);

                // isUsingLimelight = true;

                Logger.getInstance().recordOutput("AutoPlace/IndexOfPoseClosestToTarget", indexOfPoseClosestToTarget);
                if (indexOfPoseClosestToTarget != -1) {
                    LimelightTarget_Retro detection = RobotContainer.limelightVision.getRawDetections()[indexOfPoseClosestToTarget];
                    Pose3d llRobotPose = LimelightDetectionsClassifier.targetPoseToRobotPose(nodeLocation.getTranslation(), new Pose3d(RobotContainer.poseEstimator.getLatestPose()), RobotContainer.limelightVision.getRawDetections()[indexOfPoseClosestToTarget]);
                    // measurement = llRobotPose.toPose2d();

                    limelightUpdate.add(new TimestampedVisionUpdate(
                        RobotContainer.limelightVision.getAssociatedTimestamp(), 
                        llRobotPose.toPose2d(), 
                        VecBuilder.fill(0.00005, 0.00005, 1.0E6)
                    ));

                    Logger.getInstance().recordOutput("LimelightEstimatedRobotPose", llRobotPose);

                    
                    RobotContainer.poseEstimator.addVisionData(limelightUpdate);
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

        if (poseError.getTranslation().getNorm() < Units.inchesToMeters(1.0) && Math.abs(poseError.getRotation().getDegrees()) < 1.0 && !linedUp) {
            linedUp = true;
            superstructure.requestPreScore(level, isCubeNode ? GamePiece.CUBE : GamePiece.CONE);
        } 

        if (!scored) {
            if (level == Level.LOW) {
                if (superstructure.atElevatorSetpoint(ELEVATOR_PRE_LOW)) {
                    superstructure.requestScore();
                    scored = true;
                }
            } else if (isCubeNode) {
                if (superstructure.atElevatorSetpoint(level == Level.HIGH ? ELEVATOR_PRE_CUBE_HIGH : ELEVATOR_PRE_CUBE_HIGH - ELEVATOR_CUBE_OFFSET)) {
                    superstructure.requestScore();
                    scored = true;
                }
            } else if (!isCubeNode) {
                if (superstructure.atElevatorSetpoint(level == Level.HIGH ? ELEVATOR_PRE_CONE_HIGH : ELEVATOR_PRE_CONE_HIGH - ELEVATOR_CONE_OFFSET)) {
                    superstructure.requestScore();
                    scored = true;
                }
            }
        }

        swerve.requestVelocity(new ChassisSpeeds(xFeedback, yFeedback, thetaFeedback), true, false);

        Logger.getInstance().recordOutput("AutoPlace/RealGoal", realGoal);
        Logger.getInstance().recordOutput("AutoPlace/TargetRobotPose", targetRobotPose);
        Logger.getInstance().recordOutput("AutoPlace/PreTargetRobotPose", preTargetRobotPose);
        
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

}

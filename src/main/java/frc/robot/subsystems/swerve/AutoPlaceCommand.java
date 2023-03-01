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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commons.AllianceFlipUtil;
import frc.robot.commons.TunableNumber;
import frc.robot.commons.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.vision.limelight.LimelightDetectionsClassifier;

import static frc.robot.FieldConstants.*;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Limelight.*;

public class AutoPlaceCommand extends CommandBase {

    private Pose3d nodeLocation;
    private Pose2d targetRobotPose;
    private Pose2d preTargetRobotPose;
    private boolean shouldUseLimelight = false;
    private Level level;

    private final PIDController xController = new PIDController(4.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(4.0, 0.0, 0.0);
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
        int scoringLocation = RobotContainer.operatorControls.getLastSelectedScoringLocation();
        this.level = RobotContainer.operatorControls.getLastSelectedLevel();
        if (level == Level.HIGH) {
            Translation2d xyNodeTranslation = AllianceFlipUtil.apply(Grids.highTranslations[scoringLocation - 1]);
            nodeLocation = new Pose3d(
                    xyNodeTranslation.getX(),
                    xyNodeTranslation.getY(),
                    HIGH_TAPE_OFF_GROUND,
                    new Rotation3d());
        } else if (level == Level.MID) {
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
        if (level == Level.HIGH && !(scoringLocation == 2 || scoringLocation == 5 || scoringLocation == 8)) {
            targetRobotPose = nodeLocation.toPose2d().transformBy(HIGH_CONE_TO_SCORING_POS);
        } else if (level == Level.MID && !(scoringLocation == 2 || scoringLocation == 5 || scoringLocation == 8)) {
            targetRobotPose = nodeLocation.toPose2d().transformBy(MID_CONE_TO_SCORING_POS);
        } else if (level == Level.HIGH && (scoringLocation == 2 || scoringLocation == 5 || scoringLocation == 8)) {
            targetRobotPose = nodeLocation.toPose2d().transformBy(HIGH_CUBE_TO_SCORING_POS);
        } else if (level == Level.MID && (scoringLocation == 2 || scoringLocation == 5 || scoringLocation == 8)) {
            targetRobotPose = nodeLocation.toPose2d().transformBy(MID_CUBE_TO_SCORING_POS);
        } else {
            targetRobotPose = nodeLocation.toPose2d().transformBy(LOW_TO_SCORING_POS);
        }
        preTargetRobotPose = nodeLocation.toPose2d()
                .plus(new Transform2d(new Translation2d(0.5, 0.0), new Rotation2d()));
        shouldUseLimelight = !(scoringLocation == 2 || scoringLocation == 5 || scoringLocation == 8
                || level == Level.LOW);
    }

    @Override
    public void execute() {
        Pose2d realGoal = AllianceFlipUtil.apply(targetRobotPose);
        Pose2d poseError = realGoal.relativeTo(RobotContainer.poseEstimator.getLatestPose());
        if (Math.abs(poseError.getY()) > Units.inchesToMeters(3.0)) {
            realGoal = AllianceFlipUtil.apply(preTargetRobotPose);
        }

        if (shouldUseLimelight) {
            if (poseError.getTranslation().getNorm() < Units.inchesToMeters(12.0)) { // Start using limelight
                List<TimestampedVisionUpdate> limelightUpdate = new ArrayList<>();
                
                ArrayList<Pose3d> poses = RobotContainer.limelightVision.getTargets(level == Level.HIGH ? true : false);
                int indexOfPoseClosestToTarget = getIndexOfPoseClosestToTarget(poses);
                Pose3d robotPose = LimelightDetectionsClassifier.targetPoseToRobotPose(nodeLocation, RobotContainer.limelightVision.getRawDetections()[indexOfPoseClosestToTarget]);

                limelightUpdate.add(new TimestampedVisionUpdate(
                    RobotContainer.limelightVision.getAssociatedTimestamp(), 
                    robotPose.toPose2d(), 
                    VecBuilder.fill(0.005, 0.005, 0.0005)
                ));
                
                RobotContainer.poseEstimator.addVisionData(limelightUpdate);
            }
        }

        
        double xFeedback = xController.calculate(RobotContainer.poseEstimator.getLatestPose().getX(), realGoal.getX());
        double yFeedback = yController.calculate(RobotContainer.poseEstimator.getLatestPose().getY(), realGoal.getY());
        double thetaFeedback = thetaController.calculate(
            RobotContainer.poseEstimator.getLatestPose().getRotation().getRadians(),
            realGoal.getRotation().getRadians());

        xFeedback = MathUtil.clamp(xFeedback, -1, 1);
        yFeedback = MathUtil.clamp(yFeedback, -1, 1);
        thetaFeedback = MathUtil.clamp(thetaFeedback, -1.0, 1.0);

        Logger.getInstance().recordOutput("AutoPlaceGoal", realGoal);
        
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
            if (lowestError < 1.0) {
                return index;
            }
        }
        return -1;
    }

}

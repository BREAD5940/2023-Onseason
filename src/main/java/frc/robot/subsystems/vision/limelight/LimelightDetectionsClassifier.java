package frc.robot.subsystems.vision.limelight;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;
import frc.robot.commons.LimelightHelpers;
import frc.robot.commons.LoggedTunableNumber;
import frc.robot.commons.LimelightHelpers.LimelightResults;
import frc.robot.commons.LimelightHelpers.LimelightTarget_Retro;

import static frc.robot.Constants.Vision.*;

import java.util.ArrayList;

public class LimelightDetectionsClassifier extends SubsystemBase {

    private String cameraName;
    private LimelightTarget_Retro[] rawDetections;
    private double timestamp;

    LoggedTunableNumber latency = new LoggedTunableNumber("Latency", 11);

	/**
	 * set the 
	 * @param cameraName
	 */
    public LimelightDetectionsClassifier(String cameraName) {
        this.cameraName = cameraName;
    }

    public LimelightTarget_Retro[] getRawDetections() {
        return rawDetections;
    }

	/**
	 * @param lookingForTallPoles
	 * @return
	 * returns an array of Pose3ds are target pose estimates
	 */
    public ArrayList<Pose3d> getTargets(boolean lookingForTallPoles) {
        Pose3d robotPoseEstimate = new Pose3d(RobotContainer.poseEstimator.getLatestPose());
        Pose3d cameraPoseEstimate = robotPoseEstimate.transformBy(ROBOT_TO_LL); 
        Logger.getInstance().recordOutput("CameraPoseEstimate", cameraPoseEstimate);

        ArrayList<Pose3d> targets = new ArrayList<>(); // holds the actual, field-relative positions of the detections

        for (int i = 0; i < rawDetections.length; i++) { // populate the list above with targets
            double targetHeightOffGround = lookingForTallPoles ? HIGH_TAPE_OFF_GROUND : MID_TAPE_OFF_GROUND;
            double xyDistanceToTarget = getXYDistanceToTarget(rawDetections[i], targetHeightOffGround);
            double distanceToTarget = Math.hypot(xyDistanceToTarget, targetHeightOffGround - ROBOT_TO_LL.getZ());
            Logger.getInstance().recordOutput("DistanceToTargets/" + i, distanceToTarget);

            Translation3d cameraToTargetTranslation = new Translation3d(distanceToTarget, new Rotation3d(0.0, Units.degreesToRadians(-rawDetections[i].ty), Units.degreesToRadians(-rawDetections[i].tx)));
            Transform3d cameraToTargetTransform = new Transform3d(cameraToTargetTranslation, new Rotation3d()); // TODO construct the transform for transforming the camera to a target
            
            Pose3d targetPoseEstimate = cameraPoseEstimate.transformBy(cameraToTargetTransform);
            targets.add(targetPoseEstimate);
        }

        return targets;
    }

    public double getAssociatedTimestamp() {
        return timestamp;
    }

    @Override
    public void periodic() {
        long start = Logger.getInstance().getRealTimestamp();

        LimelightResults results = LimelightHelpers.getLatestResults(cameraName);
        rawDetections = results.targetingResults.targets_Retro;

        timestamp = BreadUtil.getFPGATimeSeconds() - Units.millisecondsToSeconds(LimelightHelpers.getLatency_Pipeline(cameraName) + latency.get());

        ArrayList<Pose3d> poses = getTargets(true);
        for (int i = 0; i < poses.size(); i++) {
            Logger.getInstance().recordOutput("LimelightPoses/" + i, poses.get(i));
        }

        Logger.getInstance().recordOutput("VisionAdjustedTimestamp", timestamp);

        double end = Logger.getInstance().getRealTimestamp();
        Logger.getInstance().recordOutput("LoggedRobot/LimeLightDetectionsPeriodicMs", (end-start)/1000);
    }

	/**
	 * gets the xy or 2d distance to the target
	 * @param detection // the limelight detection
	 * @param targetHeightOffGround // the expected height that the target is off the ground
	 * @return
	 * returns the distance to the target
	 */
    private static double getXYDistanceToTarget(LimelightTarget_Retro detection, double targetHeightOffGround) {
        // Define the vector
        double x = 1.0 * Math.tan(Units.degreesToRadians(detection.tx));
        double y = 1.0 * Math.tan(Units.degreesToRadians(detection.ty));
        double z = 1.0;
        double norm = Math.sqrt(x*x+y*y+z*z);
        x /= norm;
        y /= norm;
        z /= norm;

        // Rotate the vector by the camera pitch
        double xPrime = x;
        Translation2d yzPrime = new Translation2d(y, z).rotateBy(new Rotation2d(ROBOT_TO_LL.getRotation().getY()));
        double yPrime = yzPrime.getX();
        double zPrime = yzPrime.getY();

        // Solve for the intersection
        double angleToGoalRadians = Math.asin(yPrime);
        double diffHeight = targetHeightOffGround - ROBOT_TO_LL.getZ();
        double distance = diffHeight/Math.tan(angleToGoalRadians);

        return distance;
    }

    public static Pose3d targetPoseToRobotPose(Translation3d target, Pose3d robotPose, LimelightTarget_Retro detection) {
        double targetHeightOffGround = target.getZ();
        double xyDistanceToTarget = getXYDistanceToTarget(detection, targetHeightOffGround);
        double distanceToTarget = Math.hypot(xyDistanceToTarget, targetHeightOffGround - ROBOT_TO_LL.getZ());

        Pose3d estimatedCameraPose = robotPose.transformBy(ROBOT_TO_LL);        

        Translation3d cameraToTargetTranslation = new Translation3d(distanceToTarget, new Rotation3d(0.0, Units.degreesToRadians(-detection.ty), Units.degreesToRadians(-detection.tx)));
        cameraToTargetTranslation = cameraToTargetTranslation.rotateBy(estimatedCameraPose.getRotation());

        Translation3d cameraTranslation = target.minus(cameraToTargetTranslation);
        Pose3d cameraPose = new Pose3d(cameraTranslation, estimatedCameraPose.getRotation());

        Pose3d estimatedRobotPose = cameraPose.transformBy(ROBOT_TO_LL.inverse());

        return estimatedRobotPose;
    }
    
}

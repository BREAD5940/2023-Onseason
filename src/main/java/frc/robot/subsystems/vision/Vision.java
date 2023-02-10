package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import java.util.LinkedList;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import static frc.robot.FieldConstants.*;
import static frc.robot.Constants.Camera.*;

public class Vision extends SubsystemBase {

    public PhotonCamera camera = new PhotonCamera("BreadCam");
    private LinkedList<TimestampedPose2d> q = new LinkedList<>();
    private double lastTimestamp = -1;

    public Vision() { }

    public boolean hasMeasurements() {
        return !q.isEmpty();
    }   

    public TimestampedPose2d popMeasurement() {
        return q.poll();
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();
        double timestamp = result.getTimestampSeconds();
        if (result.hasTargets() && timestamp - lastTimestamp > 1.0E-6) {
            for (PhotonTrackedTarget target : result.targets) {
                Pose3d targetPosition = aprilTags.get(target.getFiducialId());
                if (targetPosition!=null) {
                    Pose2d alternateRobotToTarget = targetPosition
                        .transformBy(target.getAlternateCameraToTarget().inverse())
                        .transformBy(ROBOT_TO_CAM.inverse())
                        .toPose2d();

                    Pose2d bestRobotToTarget = targetPosition
                        .transformBy(target.getBestCameraToTarget().inverse())
                        .transformBy(ROBOT_TO_CAM.inverse())
                        .toPose2d();
                    if (target.getPoseAmbiguity() < 0.15) {
                        q.add(new TimestampedPose2d(
                            timestamp, bestRobotToTarget
                        ));
                    } else if (Math.abs(bestRobotToTarget.getRotation().minus(RobotContainer.swerve.getPose().getRotation()).getRadians()) < Math.abs(alternateRobotToTarget.getRotation().minus(RobotContainer.swerve.getPose().getRotation()).getRadians())) {
                        q.add(new TimestampedPose2d(
                            timestamp, bestRobotToTarget
                        ));
                    } else {
                        q.add(new TimestampedPose2d(
                            timestamp, alternateRobotToTarget
                        ));
                    }
                }
  
            }
            lastTimestamp = timestamp;
        }
    }

    public record TimestampedPose2d(double timestamp, Pose2d pose) { }
    
}
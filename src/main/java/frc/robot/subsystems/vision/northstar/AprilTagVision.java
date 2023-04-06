package frc.robot.subsystems.vision.northstar;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.commons.GeomUtil;
import frc.robot.commons.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.subsystems.vision.northstar.AprilTagVisionIO.AprilTagVisionIOInputs;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {
        private static final double ambiguityThreshold = 0.15;
        private static final double targetLogTimeSecs = 0.1;
        private static final Pose3d[] cameraPoses;

        private final AprilTagVisionIO[] io;
        private final AprilTagVisionIOInputs[] inputs;

        private static double xyStdDevCoefficient;
        private static double thetaStdDevCoefficient;
        private static final double fieldBorderMargin = 0.5;

        private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
        private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {};
        private Map<Integer, Double> lastDetectionTimeIds = new HashMap<>();

        static {
                cameraPoses = new Pose3d[] {
                                new Pose3d(-0.245 - Units.inchesToMeters(0.5), 0.33, 0.345,
                                                new Rotation3d(0.0, 0.0, 0.541)),
                                new Pose3d(-0.245 - Units.inchesToMeters(0.5), -0.33, 0.345,
                                                new Rotation3d(-0.055 + Units.degreesToRadians(180), -0.03, -0.541)),
                                new Pose3d(
                                        Units.inchesToMeters(6.79), Units.inchesToMeters(-3) + 0.06, Units.inchesToMeters(13.43),
                                                new Rotation3d(Units.degreesToRadians(180.0), Units.degreesToRadians(-9.5), Units.degreesToRadians(0.0))
                                )

                };
        }

        public AprilTagVision(AprilTagVisionIO... io) {
                this.io = io;
                inputs = new AprilTagVisionIOInputs[io.length];
                for (int i = 0; i < io.length; i++) {
                        inputs[i] = new AprilTagVisionIOInputs();
                }

                // Create map of last detection times
                FieldConstants.aprilTags
                                .keySet()
                                .forEach(
                                                (Integer id) -> {
                                                        lastDetectionTimeIds.put(id, 0.0);
                                                });

        }

        public void setDataInterfaces(
                        Supplier<Pose2d> poseSupplier, Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
                this.poseSupplier = poseSupplier;
                this.visionConsumer = visionConsumer;
        }

        public void periodic() {
                for (int i = 0; i < io.length; i++) {
                        io[i].updateInputs(inputs[i]);
                        Logger.getInstance().processInputs("AprilTagVision/Inst" + Integer.toString(i), inputs[i]);
                }

                // Loop over instances
                Pose2d currentPose = poseSupplier.get();
                List<Pose2d> allRobotPoses = new ArrayList<>();
                List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();
                
                for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
                        List<Pose2d> visionPose2ds = new ArrayList<>();
                        List<Pose3d> tagPose3ds = new ArrayList<>();
                        List<Integer> tagIds = new ArrayList<>();
                        
                        // Loop over frames
                        for (int frameIndex = 0; frameIndex < inputs[instanceIndex].timestamps.length; frameIndex++) {
                                var timestamp = inputs[instanceIndex].timestamps[frameIndex];
                                var values = inputs[instanceIndex].frames[frameIndex];
                                Logger.getInstance()
                                                .recordOutput(
                                                                "AprilTagVision/Inst" + Integer.toString(instanceIndex)
                                                                                + "/LatencySecs",
                                                                Timer.getFPGATimestamp() - timestamp);

                                // Switch based on number of poses
                                Pose3d cameraPose = null;
                                Pose2d robotPose = null;

                                switch ((int) values[0]) {
                                        case 1:
                                                // One pose (multi-tag), use directly
                                                cameraPose =
                                                new Pose3d(
                                                        values[2],
                                                        values[3],
                                                        values[4],
                                                        new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
                                                robotPose =
                                                cameraPose
                                                        .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                                                        .toPose2d();
                                                xyStdDevCoefficient = 0.003;
                                                thetaStdDevCoefficient = 0.0002;
                                                break;
                                        case 2:
                                                // Two poses (one tag), disambiguate
                                                double error0 = values[1];
                                                double error1 = values[9];
                                                Pose3d cameraPose0 =
                                                        new Pose3d(
                                                                values[2],
                                                                values[3],
                                                                values[4],
                                                                new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
                                                Pose3d cameraPose1 =
                                                        new Pose3d(
                                                                values[10],
                                                                values[11],
                                                                values[12],
                                                                new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
                                                Pose2d robotPose0 =
                                                        cameraPose0
                                                                .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                                                                .toPose2d();
                                                Pose2d robotPose1 =
                                                        cameraPose1
                                                                .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse())
                                                                .toPose2d();

                                                // Select pose using projection errors and current rotation
                                                if (error0 < error1 * ambiguityThreshold) {
                                                        cameraPose = cameraPose0;
                                                        robotPose = robotPose0;
                                                } else if (error1 < error0 * ambiguityThreshold) {
                                                        cameraPose = cameraPose1;
                                                        robotPose = robotPose1;
                                                } else if (Math.abs(
                                                        robotPose0.getRotation().minus(currentPose.getRotation()).getRadians())
                                                < Math.abs(
                                                        robotPose1.getRotation().minus(currentPose.getRotation()).getRadians())) {
                                                        cameraPose = cameraPose0;
                                                        robotPose = robotPose0;
                                                } else {
                                                        cameraPose = cameraPose1;
                                                        robotPose = robotPose1;
                                                }
                                                xyStdDevCoefficient = 0.01;
                                                thetaStdDevCoefficient = 0.01;
                                                break;
                                }
                                
                                // Exit if no data
                                if (cameraPose == null || robotPose == null) {
                                        continue;
                                }

                                // Exit if robot pose is off the field
                                if (robotPose.getX() < -fieldBorderMargin
                                || robotPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
                                || robotPose.getY() < -fieldBorderMargin
                                || robotPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin) {
                                continue;
                                }

                                // Get tag poses and update last detection times
                                for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i++) {
                                        Pose3d tagPose = FieldConstants.aprilTags.get((int) values[i]);
                                        int tagId = (int) values[i];

                                        tagPose3ds.add(tagPose);
                                        tagIds.add(tagId);
                                }

                                // Calculate average distance to tag
                                double totalDistance = 0.0;
                                for (Pose3d tagPose : tagPose3ds) {
                                        totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
                                }
                                double avgDistance = totalDistance / tagPose3ds.size();

                                double xyStdDev = xyStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPose3ds.size();
                                double thetaStdDev = thetaStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPose3ds.size();                                

                                visionUpdates.add(
                                new TimestampedVisionUpdate(
                                        timestamp, robotPose, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
                                allRobotPoses.add(robotPose);

                                Logger.getInstance().recordOutput("VisionData/" + instanceIndex, robotPose);
                        }

                        // Log poses
                        boolean hasFrames = inputs[instanceIndex].timestamps.length > 0;
                        if (hasFrames) {
                                Logger.getInstance()
                                .recordOutput(
                                    "AprilTagVision/RobotPoses", allRobotPoses.toArray(new Pose2d[allRobotPoses.size()]));
                                Logger.getInstance()
                                                .recordOutput(
                                                                "AprilTagVision/Inst" + Integer.toString(instanceIndex)
                                                                                + "/TagPoses",
                                                                tagPose3ds.toArray(new Pose3d[tagPose3ds.size()]));
                                Logger.getInstance()
                                                .recordOutput(
                                                                "AprilTagVision/Inst" + Integer.toString(instanceIndex)
                                                                                + "/TagIDs",
                                                                tagIds.stream().mapToLong(Long::valueOf).toArray());

                                for (int i = 0; i < visionPose2ds.size(); i++) {
                                        Logger.getInstance()
                                                        .recordOutput(
                                                                        "AprilTagVision/Inst"
                                                                                        + Integer.toString(
                                                                                                        instanceIndex)
                                                                                        + "/Poses/" + tagIds.get(i),
                                                                        visionPose2ds.get(i));
                                }
                        }
                }

                // Log target poses
                List<Pose3d> targetPose3ds = new ArrayList<>();
                for (Map.Entry<Integer, Double> detectionEntry : lastDetectionTimeIds.entrySet()) {
                        if (Timer.getFPGATimestamp() - detectionEntry.getValue() < targetLogTimeSecs) {
                                targetPose3ds.add(FieldConstants.aprilTags.get(detectionEntry.getKey()));
                        }
                }
                Logger.getInstance()
                                .recordOutput(
                                                "AprilTagVision/TargetPoses",
                                                targetPose3ds.toArray(new Pose3d[targetPose3ds.size()]));

                // Send results to pose esimator
                visionConsumer.accept(visionUpdates);
        }
}
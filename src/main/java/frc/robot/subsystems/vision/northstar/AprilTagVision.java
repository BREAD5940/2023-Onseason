package frc.robot.subsystems.vision.northstar;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.commons.GeomUtil;
import frc.robot.commons.LoggedTunableNumber;
import frc.robot.commons.PolynomialRegression;
import frc.robot.commons.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.subsystems.vision.northstar.AprilTagVisionIO.AprilTagVisionIOInputs;
import frc.robot.subsystems.vision.visionTest.AprilTagResult;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import javax.xml.crypto.dsig.Transform;

import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {
        private static final double ambiguityThreshold = 0.15;
        private static final double targetLogTimeSecs = 0.1;
        public static LoggedTunableNumber mStdDevScalar = new LoggedTunableNumber("AprilTagVision/StdDevScalar", 2.0);
        private static final Pose3d[] cameraPoses;
        private static final PolynomialRegression xyStdDevModel;
        private static final PolynomialRegression thetaStdDevModel;
        private static boolean trustHigh = false;

    private final AprilTagVisionIO[] io;
    private final AprilTagVisionIOInputs[] inputs;

    private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
    private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {
    };
    private Map<Integer, Double> lastDetectionTimeIds = new HashMap<>();
    private Map<Integer, Map<String, AprilTagResult>> latestTagReadings = new HashMap<>();;

    static {
        cameraPoses = new Pose3d[] {
                new Pose3d(-0.245 - Units.inchesToMeters(0.5), 0.33, 0.345,
                        new Rotation3d(0.0, 0.0, 0.541)),
                new Pose3d(-0.245 - Units.inchesToMeters(0.5), -0.33, 0.345,
                        new Rotation3d(-0.055 + Units.degreesToRadians(180), -0.03, -0.541)),
                new Pose3d(
                        Units.inchesToMeters(12.79), Units.inchesToMeters(-3), Units.inchesToMeters(13.43),
                        new Rotation3d(Units.degreesToRadians(180.0), Units.degreesToRadians(-9.5),
                                Units.degreesToRadians(0.0)))

        };
        xyStdDevModel = new PolynomialRegression(
                new double[] {
                        0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358,
                        2.758358,
                        3.223358, 4.093358, 4.726358
                },
                // new double[] {
                // 0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.0695, 0.046, 0.1245,
                // 0.0815, 0.193
                // },
                new double[] {
                        0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.12, 0.14, 0.17,
                        0.27, 0.38
                },
                2);
        thetaStdDevModel = new PolynomialRegression(
                new double[] {
                        0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358,
                        2.758358,
                        3.223358, 4.093358, 4.726358
                },
                new double[] {
                        0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.049, 0.027, 0.059, 0.029,
                        0.068
                },
                1);

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
        setTrustLevel(false);
    }

    public void setDataInterfaces(
            Supplier<Pose2d> poseSupplier, Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
        this.poseSupplier = poseSupplier;
        this.visionConsumer = visionConsumer;
    }

    public void periodic() {
		makeLatestTagsOld();
        long start = Logger.getInstance().getRealTimestamp();

        Logger.getInstance().recordOutput("AprilTagVision/StdDevScalar", mStdDevScalar.get());

        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.getInstance().processInputs("AprilTagVision/Inst" + Integer.toString(i), inputs[i]);
        }

        // Loop over instances
        Pose2d currentPose = poseSupplier.get();
        List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();
        for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
            List<Pose2d> visionPose2ds = new ArrayList<>();
            List<Pose3d> tagPose3ds = new ArrayList<>();
            List<Integer> tagIds = new ArrayList<>();
            String cameraIdentifier = io[instanceIndex].getIdentifier();

            // Loop over frames
            for (int frameIndex = 0; frameIndex < inputs[instanceIndex].timestamps.length; frameIndex++) {
                var timestamp = inputs[instanceIndex].timestamps[frameIndex];
                var values = inputs[instanceIndex].frames[frameIndex];
                Logger.getInstance()
                        .recordOutput(
                                "AprilTagVision/Inst" + Integer.toString(instanceIndex)
                                        + "/LatencySecs",
                                Timer.getFPGATimestamp() - timestamp);

                // Loop over observations
                for (int i = 0; i < values.length; i += 15) {
                    // Get observation data
                    int tagId = (int) values[i];
                    var pose0 = openCVPoseToWPILibPose(
                            VecBuilder.fill(values[i + 1], values[i + 2], values[i + 3]),
                            VecBuilder.fill(values[i + 4], values[i + 5], values[i + 6]));
                    var error0 = values[i + 7];
                    var pose1 = openCVPoseToWPILibPose(
                            VecBuilder.fill(values[i + 8], values[i + 9], values[i + 10]),
                            VecBuilder.fill(values[i + 11], values[i + 12],
                                    values[i + 13]));
                    var error1 = values[i + 14];

                    // Calculate robot poses
                    var fieldToTag = FieldConstants.aprilTags.get(tagId);
                    if (fieldToTag == null) {
                        continue;
                    }
                    var robotPose0 = fieldToTag
                            .transformBy(GeomUtil.pose3dToTransform3d(pose0).inverse())
                            .transformBy(GeomUtil
                                    .pose3dToTransform3d(cameraPoses[instanceIndex])
                                    .inverse());
                    // .toPose2d();
                    var robotPose1 = fieldToTag
                            .transformBy(GeomUtil.pose3dToTransform3d(pose1).inverse())
                            .transformBy(GeomUtil
                                    .pose3dToTransform3d(cameraPoses[instanceIndex])
                                    .inverse());
                    // .toPose2d();

                    // Choose better pose
                    Pose3d robotPose3d;
                    Pose2d robotPose;
                    Pose3d tagPose;
                    if (error0 < error1 * ambiguityThreshold) {
                        robotPose = robotPose0.toPose2d();
                        robotPose3d = robotPose0;
                        tagPose = pose0;
                    } else if (error1 < error0 * ambiguityThreshold) {
                        robotPose = robotPose1.toPose2d();
                        robotPose3d = robotPose1;
                        tagPose = pose1;
                    } else if (Math.abs(
                            robotPose0.toPose2d().getRotation()
                                    .minus(currentPose.getRotation())
                                    .getRadians()) < Math
                                            .abs(robotPose1.toPose2d()
                                                    .getRotation()
                                                    .minus(currentPose
                                                            .getRotation())
                                                    .getRadians())) {
                        robotPose = robotPose0.toPose2d();
                        robotPose3d = robotPose0;
                        tagPose = pose0;
                    } else {
                        robotPose = robotPose1.toPose2d();
                        tagPose = pose1;
                        robotPose3d = robotPose1;
                    }

                    /* Measurements of camera transforms */
                    // Transform3d tagToRobot = new Transform3d(new Translation3d(0.7096, 1.1261,
                    // -Units.inchesToMeters(18.22)), new Rotation3d(0.0, 0.0, Math.PI));
                    // Pose3d robotToField =
                    // FieldConstants.aprilTags.get(7).transformBy(tagToRobot);
                    // Transform3d robotToCam = new Transform3d(robotToField, robotPose3d);
                    // Logger.getInstance().recordOutput("Pose0", robotPose0);
                    // Logger.getInstance().recordOutput("Pose1", robotPose1);
                    // Logger.getInstance().recordOutput("Robot To Field", robotToField);
                    // Logger.getInstance().recordOutput("Camera To Field", robotPose3d);
                    // Logger.getInstance().recordOutput("CameraToFieldRoll",
                    // robotToCam.getRotation().getX());
                    // Logger.getInstance().recordOutput("CameraToFieldPitch",
                    // robotToCam.getRotation().getY());
                    // Logger.getInstance().recordOutput("CameraToFieldYaw",
                    // robotToCam.getRotation().getZ());
                    // Logger.getInstance().recordOutput("Robot To Cam",
                    // GeomUtil.transform3dToPose3d(robotToCam));

                    // Set latest tag pose per input
                    /*if (!latestTagReadings.containsKey(tagId)) {
                        latestTagReadings.put(tagId, new HashMap<String, AprilTagResult>());
                    }
                    latestTagReadings.get(tagId).put(cameraIdentifier, new AprilTagResult(
                            error0,
                            cameraPoses[instanceIndex].transformBy(GeomUtil.pose3dToTransform3d(pose0)),
                            error1,
                            cameraPoses[instanceIndex].transformBy(GeomUtil.pose3dToTransform3d(pose1)),
                            timestamp));*/

                    // Log tag pose
                    tagPose3ds.add(tagPose);
                    tagIds.add(tagId);
                    lastDetectionTimeIds.put(tagId, Timer.getFPGATimestamp());

                    // Add to vision updates
                    double tagDistance = tagPose.getTranslation().getNorm();
                    double xyStdDev = xyStdDevModel.predict(tagDistance) * mStdDevScalar.get();
                    double thetaStdDev = thetaStdDevModel.predict(tagDistance) * mStdDevScalar.get();
                    visionUpdates.add(
                            new TimestampedVisionUpdate(
                                    timestamp, robotPose, VecBuilder.fill(xyStdDev,
                                            xyStdDev, thetaStdDev)));
                    visionPose2ds.add(robotPose);
                    Logger.getInstance().recordOutput("VisionData/" + instanceIndex, robotPose);
                }
            }

            // Log poses
            boolean hasFrames = inputs[instanceIndex].timestamps.length > 0;
            if (hasFrames) {
                Logger.getInstance()
                        .recordOutput(
                                "AprilTagVision/Inst" + Integer.toString(instanceIndex)
                                        + "/RobotPoses",
                                visionPose2ds.toArray(
                                        new Pose2d[visionPose2ds.size()]));
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

        double end = Logger.getInstance().getRealTimestamp();
        Logger.getInstance().recordOutput("LoggedRobot/AprilTagPeriodicMs", (end - start) / 1000);
    }

    private static Pose3d openCVPoseToWPILibPose(Vector<N3> tvec, Vector<N3> rvec) {
        return new Pose3d(
                new Translation3d(tvec.get(2, 0), -tvec.get(0, 0), -tvec.get(1, 0)),
                new Rotation3d(
                        VecBuilder.fill(rvec.get(2, 0), -rvec.get(0, 0), -rvec.get(1, 0)),
                        Math.sqrt(
                                Math.pow(rvec.get(0, 0), 2)
                                        + Math.pow(rvec.get(1, 0), 2)
                                        + Math.pow(rvec.get(2, 0), 2))));
    }

    public static void setTrustLevel(boolean isTrustHigh) {
        //mStdDevScalar = isTrustHigh ? 0.2 : 2.0;
    }

    public Map<Integer, Map<String, AprilTagResult>> getLatestTagReadings() {
        return latestTagReadings;
    }

    private void makeLatestTagsOld() {
        for (Map<String, AprilTagResult> tagReading : latestTagReadings.values()) {
            for (AprilTagResult result : tagReading.values()) {
                result.isOld = true;
            }
        }
    }
}

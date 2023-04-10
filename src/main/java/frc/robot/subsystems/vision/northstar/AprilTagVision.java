package frc.robot.subsystems.vision.northstar;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
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
import frc.robot.subsystems.vision.visionTest.RobotPose;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {
        public static LoggedTunableNumber mStdDevScalar = new LoggedTunableNumber("AprilTagVision/StdDevScalar", 2.0);
        private static final double ambiguityThreshold = 0.15;
        private static final double targetLogTimeSecs = 0.1;
        private static final Pose3d[] cameraPoses;

        private final AprilTagVisionIO[] io;
        private final AprilTagVisionIOInputs[] inputs;

        private double xyStdDevCoefficient;
        private double thetaStdDevCoefficient;
        private static final double fieldBorderMargin = 0.5;
        private static final PolynomialRegression xyStdDevModel;
        private static final PolynomialRegression thetaStdDevModel;

        private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
        private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {};
        private Map<Integer, Double> lastDetectionTimeIds = new HashMap<>();
        private Pose2d currentPose;
        private List<TimestampedVisionUpdate> visionUpdates;
        private List<Pose2d> allRobotPoses;
		private Map<String, RobotPose> mostResentRobotPoses = new HashMap<>();

        static {

				//gamma
				cameraPoses = new Pose3d[] {
					new Pose3d(-0.245 - Units.inchesToMeters(0.5), 0.33, 0.345,
									new Rotation3d(0.0, 0.0, 0.541)),
					new Pose3d(-0.245 - Units.inchesToMeters(0.5), -0.33, 0.345,
									new Rotation3d(-0.055 + Units.degreesToRadians(180), -0.03, -0.541)),
					new Pose3d(
							Units.inchesToMeters(6.79), Units.inchesToMeters(-3) + 0.06, Units.inchesToMeters(13.43),
									new Rotation3d(Units.degreesToRadians(180.0), Units.degreesToRadians(-9.5), Units.degreesToRadians(0.0))
					)};
				
				//beta
				/*cameraPoses = new Pose3d[] {
					new Pose3d(-0.245, 0.33, 0.345,
									new Rotation3d(3.031, 0.049, 0.593)),
					new Pose3d(-0.245, -0.33, 0.345,
									new Rotation3d(-0.055, -0.03, -0.541)),
					new Pose3d(
							Units.inchesToMeters(8.640), Units.inchesToMeters(-5.163), Units.inchesToMeters(12.209),
									new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-25.0), Units.degreesToRadians(0.0))
					)};*/
				
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

        }

        public void setDataInterfaces(
                        Supplier<Pose2d> poseSupplier, Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
                this.poseSupplier = poseSupplier;
                this.visionConsumer = visionConsumer;
        }

        public void periodic() {
				mostResentRobotPoses = new HashMap<>();
                Logger.getInstance().recordOutput("AprilTagVision/StdDevScalar", mStdDevScalar.get());

                for (int i = 0; i < io.length; i++) {
                        io[i].updateInputs(inputs[i]);
                        Logger.getInstance().processInputs("AprilTagVision/Inst" + Integer.toString(i), inputs[i]);
                }

                // Loop over instances
                currentPose = poseSupplier.get();
                allRobotPoses = new ArrayList<>();
                visionUpdates = new ArrayList<>();
                
                for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
                        List<Pose2d> visionPose2ds = new ArrayList<>();
                        List<Pose3d> tagPose3ds = new ArrayList<>();
                        List<Integer> tagIds = new ArrayList<>();
                        
                        // Loop over frames
                        for (int frameIndex = 0; frameIndex < inputs[instanceIndex].timestamps.length; frameIndex++) {
                                var timestamp = inputs[instanceIndex].timestamps[frameIndex];
                                var values = inputs[instanceIndex].frames[frameIndex];
                                int version = 2;//(int) inputs[instanceIndex].version;

                                Logger.getInstance()
                                                .recordOutput(
                                                                "AprilTagVision/Inst" + Integer.toString(instanceIndex)
                                                                                + "/LatencySecs",
                                                                Timer.getFPGATimestamp() - timestamp);
                                Pose3d cameraPose = null;

                                // Default value (100) represents an unloaded value
                                if (version == 100) {
                                        continue;
                                }

                                switch (version) {
                                        case 1:
                                                if (values.length < 15) {
                                                        continue;
                                                }

                                                processVersion1(values, instanceIndex, tagPose3ds, tagIds, visionPose2ds, timestamp);
                                                break;
                                        case 2:
                                                if (values.length < 8) {
                                                        continue;
                                                }
                                        
                                                processVersion2(values, instanceIndex, cameraPose, tagPose3ds, tagIds, timestamp);
                                                break;
                                        default:
                                                if (values.length < 15) {
                                                        continue;
                                                }
                                                // Default to version 1 if "version" is not 1 or 2
                                                processVersion1(values, instanceIndex, tagPose3ds, tagIds, visionPose2ds, timestamp);
                                                break;
                                }
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

        public static void setTrustLevel(boolean isTrustHigh) {
                // mStdDevScalar = isTrustHigh ? 0.2 : 2.0;
        }

        public void processVersion1(double[] values, int instanceIndex, List<Pose3d> tagPose3ds, List<Integer> tagIds, List<Pose2d> visionPose2ds, double timestamp) {
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
                        var robotPose1 = fieldToTag
                                        .transformBy(GeomUtil.pose3dToTransform3d(pose1).inverse())
                                        .transformBy(GeomUtil
                                                        .pose3dToTransform3d(cameraPoses[instanceIndex])
                                                        .inverse());

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

                        // Exit if no data
                        if (robotPose == null) {
                                continue;
                        }

                        // Exit if robot pose is off the field
                        if (robotPose.getX() < -fieldBorderMargin
                        || robotPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
                        || robotPose.getY() < -fieldBorderMargin
                        || robotPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin) {
                                continue;
                        }


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

        public void processVersion2(double[] values, int instanceIndex, Pose3d cameraPose, List<Pose3d> tagPose3ds, List<Integer> tagIds, double timestamp) {
                Pose2d robotPose = null;
				Pose3d robotPose3d = null; // robot pose for fault checker
				Pose3d robotPose3dAlt = null; // alt robot pose for fault checker

                switch ((int) values[0]) {
                        case 1:
                                // One pose (multi-tag), use directly
                                cameraPose =
                                new Pose3d(
                                        values[2],
                                        values[3],
                                        values[4],
                                        new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
                                robotPose3d = cameraPose.transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse());
								robotPose = robotPose3d.toPose2d();
                                xyStdDevCoefficient = 0.003;
                                thetaStdDevCoefficient = 0.002;
                                Logger.getInstance().recordOutput("Camera Pose (Multi Tag) " + instanceIndex, cameraPose);
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
                                Pose3d robotPose0 =
                                        cameraPose0
                                                .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse());
                                Pose3d robotPose1 =
                                        cameraPose1
                                                .transformBy(GeomUtil.pose3dToTransform3d(cameraPoses[instanceIndex]).inverse());

                                // Select pose using projection errors and current rotation
                                if (error0 < error1 * ambiguityThreshold) {
                                        cameraPose = cameraPose0;
                                        robotPose = robotPose0.toPose2d();
										robotPose3d = robotPose0;
                                } else if (error1 < error0 * ambiguityThreshold) {
                                        cameraPose = cameraPose1;
                                        robotPose = robotPose1.toPose2d();
										robotPose3d = robotPose1;
                                } else if (Math.abs(
                                        robotPose0.toPose2d().getRotation().minus(currentPose.getRotation()).getRadians())
                                < Math.abs(
                                        robotPose1.toPose2d().getRotation().minus(currentPose.getRotation()).getRadians())) {
                                        cameraPose = cameraPose0;
                                        robotPose = robotPose0.toPose2d();
                                } else {
                                        cameraPose = cameraPose1;
                                        robotPose = robotPose1.toPose2d();
                                }
                                Logger.getInstance().recordOutput("Camera Pose (Single Tag) " + instanceIndex, cameraPose);
								if (robotPose3d == null) {
									robotPose3d = robotPose0;
									robotPose3dAlt = robotPose1;
								}
                                break;
                }
                System.out.println(robotPose);

                // Exit if no data
                if (cameraPose == null || robotPose == null) {
                        return;
                }

				//gets the robot for the fault checker
				if (robotPose3d != null) {
					mostResentRobotPoses.put(io[instanceIndex].getIdentifier(), new RobotPose(robotPose3d, robotPose3dAlt));
				}

                // Exit if robot pose is off the field
                if (robotPose.getX() < -fieldBorderMargin
                || robotPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
                || robotPose.getY() < -fieldBorderMargin
                || robotPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin) {
                        return;
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
                double xyStdDev = 0.0;
                double thetaStdDev = 0.0;

                switch ((int) values[0]) {
                        case 1:
                                xyStdDev = xyStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPose3ds.size();
                                thetaStdDev = thetaStdDevCoefficient * Math.pow(avgDistance, 2.0) / tagPose3ds.size(); 
                                break;
                        case 2:
                                xyStdDev = xyStdDevModel.predict(avgDistance) * mStdDevScalar.get();
                                thetaStdDev = thetaStdDevModel.predict(avgDistance) * mStdDevScalar.get();
                                break;
                }                           

                visionUpdates.add(
                new TimestampedVisionUpdate(
                        timestamp, robotPose, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
                allRobotPoses.add(robotPose);

                Logger.getInstance().recordOutput("VisionData/" + instanceIndex, robotPose);
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


		/** gets the poses for the faultchecker */
		public Map<String, RobotPose> getRobotPosesForFaultChecker() {
			return mostResentRobotPoses;
		}
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.visionTest;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.northstar.AprilTagVision;

/** Used to test if the camera on the robot are in the correct positions */
public class CameraPoseTester {
	private Map<String, Pose3d> poseHack = new HashMap<>();
    private AprilTagVision visionSupplier;
    private double visionTesterTolerance = 0.15; // meters
    private Map<Pair<String, String>, Boolean> cameraAlignment = new HashMap<>();

	private Map<String, Pair<LinearFilter, AlignmentTypes>> activeAlignmentsCheck = new HashMap<String, Pair<LinearFilter, AlignmentTypes>>();

    public CameraPoseTester(AprilTagVision visionSupplier) {
        this.visionSupplier = visionSupplier;
		poseHack.put(RobotContainer.centerCamera.getIdentifier(), new Pose3d(
			new Translation3d(-6.48, -0.83, 8.43),
			new Rotation3d(new Quaternion(-0.09410561680015958, 0.9705204381281072, 0.011474785146114441, -0.22159093250433082))));
    }

    /** runs tests on the all cameras and then holds the result internally */
    public void update() {
    	Map<String, Pose3d> allPoseReadings = getTagData();
		Map<Pair<String, String>, Boolean> camAlignment = new HashMap<>();
		String[] allPosesKeys = allPoseReadings.keySet().toArray(new String[0]);
		for (int i = 0; i < allPosesKeys.length; i++) {
			for (int j = i+1; j < allPosesKeys.length; j++) {
				Logger.getInstance().recordOutput("faultChecker/vision/lastUpdate", Timer.getFPGATimestamp());
				camAlignment.put(
					new Pair<String,String>(allPosesKeys[i], allPosesKeys[j]),
					areAligned(allPoseReadings.get(allPosesKeys[i]), allPoseReadings.get(allPosesKeys[j])));
			}
		}

        cameraAlignment = camAlignment;
    }

	public enum AlignmentTypes {
		ALIGNED,
		NOT_ALIGNED,
		NO_RESULT,
	}

    /**
     * @param cam1
     * @param cam2
     * @return 
	 * This will retrun AlignmentTypes.ALIGNED when the tags are aligned,
	 * AlignmentTypes.NOT_ALIGNED when the tags are not aligned,
	 * and AlignmentTypes.NO_RESULT when there is not enough data to choose beteen the tags.
	 */
    private AlignmentTypes checkCameraAlignment(String cam1, String cam2) {
		for (Pair<String,String> key : cameraAlignment.keySet()) {
			if ((key.getFirst() == cam1 && key.getSecond() == cam2) || (key.getFirst() == cam2 && key.getSecond() == cam1)) {
				return cameraAlignment.get(key) ?
					AlignmentTypes.ALIGNED : AlignmentTypes.NOT_ALIGNED;
			}
		}
		// System.out.printf("Could not get alignment data for cameras %S and %S. %n", cam1, cam2);

		return AlignmentTypes.NO_RESULT;
    }

    /**
     * @param pose0
     * @param pose1
     * @return retruns true if the poses are aligned
     */
    private boolean areAligned(Pose3d pose0, Pose3d pose1) {
        // this will do a distance only based check
        double dis = pose0.getTranslation().getDistance(pose1.getTranslation());
		Logger.getInstance().recordOutput("faultChecker/vision/tagDis", dis);
		Logger.getInstance().recordOutput("faultChecker/vision/pose0", pose0);
		Logger.getInstance().recordOutput("faultChecker/vision/pose1", pose1);

        return dis < visionTesterTolerance; 
    }

    /**
     * @return This will retrun a map that has all the latest tag pose estimates.
     */
    private Map<String, Pose3d> getTagData() {
       	Map<String, RobotPose> allCameraPoseReadings = visionSupplier.getRobotPosesForFaultChecker();
        Map<String, Pose3d> allPoseReadings = new HashMap<>();
		if (allCameraPoseReadings.size() <= 1) {
			return allPoseReadings;
		}
        for (String cameraIdentifier : allCameraPoseReadings.keySet()) {
			if (allPoseReadings.containsKey(cameraIdentifier)) {
				continue;
			}
			if (allCameraPoseReadings.get(cameraIdentifier).poseAlt != null) {
				RobotPose robotPose = allCameraPoseReadings.get(cameraIdentifier);
				for (String cameraIdentifier2 : allCameraPoseReadings.keySet()) {
					if (cameraIdentifier == cameraIdentifier2) {
						continue;
					}
					if (allCameraPoseReadings.get(cameraIdentifier2).poseAlt != null) {
						Logger.getInstance().recordOutput("faultChecker/vision/didCompare", true);
						RobotPose robotPose2 = allCameraPoseReadings.get(cameraIdentifier2);
						double compare00 = robotPose.pose.getTranslation().getDistance(robotPose2.pose.getTranslation());
						double compare10 = robotPose.poseAlt.getTranslation().getDistance(robotPose2.pose.getTranslation());
						double compare01 = robotPose.pose.getTranslation().getDistance(robotPose2.poseAlt.getTranslation());
						double compare11 = robotPose.poseAlt.getTranslation().getDistance(robotPose2.poseAlt.getTranslation());
						Logger.getInstance().recordOutput("faultChecker/vision/compare00", compare00);
						Logger.getInstance().recordOutput("faultChecker/vision/compare10", compare10);
						Logger.getInstance().recordOutput("faultChecker/vision/compare01", compare01);
						Logger.getInstance().recordOutput("faultChecker/vision/compare11", compare11);
						Logger.getInstance().recordOutput("faultChecker/vision/compare0_", robotPose.pose);
						Logger.getInstance().recordOutput("faultChecker/vision/compare1_", robotPose.poseAlt);
						Logger.getInstance().recordOutput("faultChecker/vision/compare_0", robotPose2.pose);
						Logger.getInstance().recordOutput("faultChecker/vision/compare_1", robotPose2.poseAlt);

						if (compare00 < compare10 && compare00 < compare01 && compare00 < compare11) {
							allPoseReadings.put(cameraIdentifier, prepPoseForCheck(robotPose.pose, cameraIdentifier));
							allPoseReadings.put(cameraIdentifier2, prepPoseForCheck(robotPose2.pose, cameraIdentifier2));
						} else if (compare10 < compare01 && compare10 < compare11) {
							allPoseReadings.put(cameraIdentifier, prepPoseForCheck(robotPose.poseAlt, cameraIdentifier));
							allPoseReadings.put(cameraIdentifier2, prepPoseForCheck(robotPose2.poseAlt, cameraIdentifier2));
						} else if (compare01 < compare11) {
							allPoseReadings.put(cameraIdentifier, prepPoseForCheck(robotPose.pose, cameraIdentifier));
							allPoseReadings.put(cameraIdentifier2, prepPoseForCheck(robotPose2.pose, cameraIdentifier2));
						} else {
							allPoseReadings.put(cameraIdentifier, prepPoseForCheck(robotPose.poseAlt, cameraIdentifier));
							allPoseReadings.put(cameraIdentifier2, prepPoseForCheck(robotPose2.poseAlt, cameraIdentifier2));
						}
					} else {
						Logger.getInstance().recordOutput("faultChecker/vision/didCompare", false);
						Pose3d pose2 = allCameraPoseReadings.get(cameraIdentifier2).pose;
						if (pose2.getTranslation().getDistance(robotPose.pose.getTranslation()) <
						pose2.getTranslation().getDistance(robotPose.poseAlt.getTranslation())) {
							allPoseReadings.put(cameraIdentifier, prepPoseForCheck(robotPose.pose, cameraIdentifier));
						} else {
							allPoseReadings.put(cameraIdentifier, prepPoseForCheck(robotPose.poseAlt, cameraIdentifier));
						}
					}
				}
			} else {
				Logger.getInstance().recordOutput("faultChecker/vision/didCompare", false);
				allPoseReadings.put(cameraIdentifier, prepPoseForCheck(allCameraPoseReadings.get(cameraIdentifier).pose, cameraIdentifier));
			}
        }
		for (String id : allPoseReadings.keySet()) {
			for (String id2 : allPoseReadings.keySet()) {
				System.out.println(id + "relative to" + id2 + allPoseReadings.get(id).relativeTo(allPoseReadings.get(id2)));
			}
		}


        return allPoseReadings;
    }

	/**
	 * Used to start a alignment check. Use updateAlignmentCheck with the same inputs to update and read the check.
	 * @param cam1
	 * @param cam2
	 * @param tagId
	 */
	public void startAlignmentCheck(String cam1, String cam2) {
		activeAlignmentsCheck.put(cam1 + ", " + cam2, new Pair<LinearFilter, AlignmentTypes>(LinearFilter.movingAverage(10), AlignmentTypes.NO_RESULT));
	}

	/**
	 * Used to update a alignment check every robot loop. Create a alignment check with startAlignmentCheck.
	 * @param cam1
	 * @param cam2
	 * @param tagId
	 * @return
	 * Returns AlignmentTypes.NO_RESULT until the check has gotten enough data to give a result.
	 * The result will allways be AlignmentTypes.ALIGNED or AlignmentTypes.NOT_ALIGNED
	 */
	public AlignmentTypes updateAlignmentCheck(String cam1, String cam2) {
		if (activeAlignmentsCheck.containsKey(cam1 + ", " + cam2)) {
			AlignmentTypes alignment = checkCameraAlignment(cam1, cam2);
			if (alignment == AlignmentTypes.ALIGNED) {
				activeAlignmentsCheck.put(cam1 + ", " + cam2, new Pair<LinearFilter, AlignmentTypes>(
					activeAlignmentsCheck.get(cam1 + ", " + cam2).getFirst(),
					activeAlignmentsCheck.get(cam1 + ", " + cam2).getFirst().calculate(1) > 0.8 ?
						AlignmentTypes.ALIGNED : AlignmentTypes.NOT_ALIGNED));
			} else if (alignment == AlignmentTypes.NOT_ALIGNED) {
				activeAlignmentsCheck.put(cam1 + ", " + cam2, new Pair<LinearFilter, AlignmentTypes>(
					activeAlignmentsCheck.get(cam1 + ", " + cam2).getFirst(),
					activeAlignmentsCheck.get(cam1 + ", " + cam2).getFirst().calculate(0) > 0.8 ?
						AlignmentTypes.ALIGNED : AlignmentTypes.NOT_ALIGNED));
			}
			return activeAlignmentsCheck.get(cam1 + ", " + cam2).getSecond();
		}
		return AlignmentTypes.NO_RESULT;
	}

	private Pose3d prepPoseForCheck(Pose3d pose, String cameraIdentifier) {
		Pose3d tagPose = FieldConstants.aprilTags.get(5);
		/*if (poseHack.containsKey(cameraIdentifier)) {
			pose = tagPose.relativeTo(pose.transformBy(new Transform3d(
				poseHack.get(cameraIdentifier).getTranslation(),
				poseHack.get(cameraIdentifier).getRotation())));
		}*/
		return pose;
	}
}


/*
northstar-centerrelative tonorthstar-leftPose3d(Translation3d(X: 9.50, Y: 0.64, Z: 4.80), Rotation3d(Quaternion(-0.09410561680015958, -0.9705204381281072, -0.011474785146114441, 0.22159093250433082)))
northstar-leftrelative tonorthstar-centerPose3d(Translation3d(X: -6.48, Y: -0.83, Z: 8.43), Rotation3d(Quaternion(-0.09410561680015958, 0.9705204381281072, 0.011474785146114441, -0.22159093250433082)))
 */
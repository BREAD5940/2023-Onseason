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
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.northstar.AprilTagVision;

/** Used to test if the camera on the robot are in the correct positions */
public class CameraPoseTester {
    private AprilTagVision visionSupplier;
    private double visionTesterTolerance = 0.1;
    private Map<Pair<String, String>, Boolean> cameraAlignment = new HashMap<>();

	private Map<String, Pair<LinearFilter, AlignmentTypes>> activeAlignmentsCheck = new HashMap<String, Pair<LinearFilter, AlignmentTypes>>();

    public CameraPoseTester(AprilTagVision visionSupplier) {
        this.visionSupplier = visionSupplier;
    }

    /** runs tests on the all cameras and then holds the result internally */
    public void update() {
    	Map<String, Pose3d> allPoseReadings = getTagData();
		Map<Pair<String, String>, Boolean> camAlignment = new HashMap<>();
		String[] allPosesKeys = allPoseReadings.keySet().toArray(new String[0]);
		for (int i = 0; i < allPosesKeys.length; i++) {
			for (int j = i+1; j < allPosesKeys.length; j++) {
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
	 * @param tagId
     * @return 
	 * This will retrun AlignmentTypes.ALIGNED when the tags are aligned,
	 * AlignmentTypes.NOT_ALIGNED when the tags are not aligned,
	 * and AlignmentTypes.NO_RESULT when there is not enough data to choose beteen the tags.
	 */
    public AlignmentTypes checkCameraAlignment(String cam1, String cam2, Integer tagId) {
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
     * @param tagPose0
     * @param tagPose1
     * @return retruns true if the poses are aligned
     */
    private boolean areAligned(Pose3d tagPose0, Pose3d tagPose1) {
        // this will do a distance only based check
        double dis = tagPose0.getTranslation().getDistance(tagPose1.getTranslation());
		Logger.getInstance().recordOutput("cameraTest/tagDis", dis);
        return dis < visionTesterTolerance; 
    }

    /**
     * @return This will retrun a map that has all the latest tag pose estimates.
     */
    private Map<String, Pose3d> getTagData() {
       	Map<String, RobotPose> allCameraPoseReadings = visionSupplier.getRobotPosesForFaultChecker();
        Map<String, Pose3d> allPoseReadings = new HashMap<>();
        for (String cameraIdentifier : allCameraPoseReadings.keySet()) {
			if (allCameraPoseReadings.get(cameraIdentifier).poseAlt != null) {
				RobotPose robotPose = allCameraPoseReadings.get(cameraIdentifier);
				for (String cameraIdentifier2 : allCameraPoseReadings.keySet()) {
					if (cameraIdentifier != cameraIdentifier2) {
						if (allCameraPoseReadings.get(cameraIdentifier2).poseAlt != null) {
							RobotPose robotPose2 = allCameraPoseReadings.get(cameraIdentifier2);
							double compare00 = robotPose.pose.getTranslation().getDistance(robotPose2.pose.getTranslation());
							double compare10 = robotPose.poseAlt.getTranslation().getDistance(robotPose2.pose.getTranslation());
							double compare01 = robotPose.pose.getTranslation().getDistance(robotPose2.poseAlt.getTranslation());
							double compare11 = robotPose.poseAlt.getTranslation().getDistance(robotPose2.poseAlt.getTranslation());

							if (compare00 > compare10 && compare00 > compare01 && compare00 > compare11) {
								allPoseReadings.put(cameraIdentifier, inversPose3d(robotPose.pose));
							} else if (compare10 > compare01 && compare10 > compare11) {
								allPoseReadings.put(cameraIdentifier, inversPose3d(robotPose.poseAlt));
							} else if (compare01 > compare11) {
								allPoseReadings.put(cameraIdentifier, inversPose3d(robotPose.pose));
							} else {
								allPoseReadings.put(cameraIdentifier, inversPose3d(robotPose.poseAlt));
							}
						} else {
							Pose3d pose2 = allCameraPoseReadings.get(cameraIdentifier2).pose;
							if (pose2.getTranslation().getDistance(robotPose.pose.getTranslation()) <
							pose2.getTranslation().getDistance(robotPose.poseAlt.getTranslation())) {
								allPoseReadings.put(cameraIdentifier, inversPose3d(robotPose.pose));
							} else {
								allPoseReadings.put(cameraIdentifier, inversPose3d(robotPose.poseAlt));
							}
						}
					}
				}
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
	public void startAlignmentCheck(String cam1, String cam2, int tagId) {
		activeAlignmentsCheck.put(cam1 + ", " + cam2 + ", " + tagId, new Pair<LinearFilter, AlignmentTypes>(LinearFilter.movingAverage(10), AlignmentTypes.NO_RESULT));
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
	public AlignmentTypes updateAlignmentCheck(String cam1, String cam2, int tagId) {
		if (activeAlignmentsCheck.containsKey(cam1 + ", " + cam2 + ", " + tagId)) {
			AlignmentTypes alignment = checkCameraAlignment(cam1, cam2, tagId);
			if (alignment == AlignmentTypes.ALIGNED) {
				activeAlignmentsCheck.put(cam1 + ", " + cam2 + ", " + tagId, new Pair<LinearFilter, AlignmentTypes>(
					activeAlignmentsCheck.get(cam1 + ", " + cam2 + ", " + tagId).getFirst(),
					activeAlignmentsCheck.get(cam1 + ", " + cam2 + ", " + tagId).getFirst().calculate(1) > 0.8 ?
						AlignmentTypes.ALIGNED : AlignmentTypes.NOT_ALIGNED));
			} else if (alignment == AlignmentTypes.NOT_ALIGNED) {
				activeAlignmentsCheck.put(cam1 + ", " + cam2 + ", " + tagId, new Pair<LinearFilter, AlignmentTypes>(
					activeAlignmentsCheck.get(cam1 + ", " + cam2 + ", " + tagId).getFirst(),
					activeAlignmentsCheck.get(cam1 + ", " + cam2 + ", " + tagId).getFirst().calculate(0) > 0.8 ?
						AlignmentTypes.ALIGNED : AlignmentTypes.NOT_ALIGNED));
			}
			return activeAlignmentsCheck.get(cam1 + ", " + cam2 + ", " + tagId).getSecond();

		}
		return AlignmentTypes.NO_RESULT;
	}

	private Pose3d inversPose3d(Pose3d pose) {
		Transform3d transform3d = new Transform3d(pose.getTranslation(), pose.getRotation()).inverse();
		return new Pose3d(transform3d.getTranslation(), transform3d.getRotation());
	}
}


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
import frc.robot.subsystems.vision.northstar.AprilTagVision;

/** Used to test if the camera on the robot are in the correct positions */
public class CameraPoseTester {
    private AprilTagVision visionSupplier;
    private double visionTesterTolerance = 0.1;
    private Map<Integer, Map<Pair<String, String>, Boolean>> cameraAlignment = new HashMap<Integer, Map<Pair<String, String>, Boolean>>();

	private Map<String, Pair<LinearFilter, AlignmentTypes>> activeAlignmentsCheck = new HashMap<String, Pair<LinearFilter, AlignmentTypes>>();

    public CameraPoseTester(AprilTagVision visionSupplier) {
        this.visionSupplier = visionSupplier;
    }

    /** runs tests on the all cameras and then holds the result internally */
    public void update() {
        Map<Integer, Map<String, Pose3d>> allPoseReadings = getTagData();
        Map<Integer, Map<Pair<String, String>, Boolean>> camAlignment = new HashMap<Integer, Map<Pair<String, String>, Boolean>>();
        for (Integer tagId : allPoseReadings.keySet()) {
            Map<String, Pose3d> tagPoses = allPoseReadings.get(tagId);
            String[] tagPosesKeys = tagPoses.keySet().toArray(new String[0]);
            for (int i = 0; i < tagPosesKeys.length; i++) {
                for (int j = i+1; j < tagPosesKeys.length; j++) {
					if (!camAlignment.containsKey(tagId)) {
						camAlignment.put(tagId, new HashMap<Pair<String, String>, Boolean>());
					}
                    camAlignment.get(tagId).put(
                        new Pair<String,String>(tagPosesKeys[i], tagPosesKeys[j]),
                        areAligned(tagPoses.get(tagPosesKeys[i]), tagPoses.get(tagPosesKeys[j])));
                }
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
		if (cameraAlignment.containsKey(tagId)) {
			for (Pair<String,String> key : cameraAlignment.get(tagId).keySet()) {
				if ((key.getFirst() == cam1 && key.getSecond() == cam2) || (key.getFirst() == cam2 && key.getSecond() == cam1)) {
					return cameraAlignment.get(tagId).get(key) ?
						AlignmentTypes.ALIGNED : AlignmentTypes.NOT_ALIGNED;
				}
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
    private Map<Integer, Map<String, Pose3d>> getTagData() {
        Map<Integer, Map<String, AprilTagResult>> allTagReadings = visionSupplier.getLatestTagReadings();
		System.out.println(allTagReadings);
        Map<Integer, Map<String, Pose3d>> allPoseReadings = new HashMap<Integer, Map<String, Pose3d>>();
        for (Integer tagId : allTagReadings.keySet()) {
            Map<String, AprilTagResult> tagReadings = allTagReadings.get(tagId);
            allPoseReadings.put(tagId, new HashMap<String, Pose3d>());
			Map<String, Pose3d> poseReadings = allPoseReadings.get(tagId);
            for (String cameraIdentifier : tagReadings.keySet()) {
                AprilTagResult tagResult = tagReadings.get(cameraIdentifier);
                if (!tagResult.isOld) {
					Logger.getInstance().recordOutput("tester/" + tagId + cameraIdentifier, true);
                    if (tagResult.bestPose == null) {
                        poseReadings.put(cameraIdentifier, null); // will get replaced later
                    } else {
                        poseReadings.put(cameraIdentifier, tagResult.bestPose);
                    }
                } else {
					Logger.getInstance().recordOutput("tester/" + tagId + cameraIdentifier, false);
				}
            }
			
			for (String cameraIdentifier : poseReadings.keySet()) {
				if (poseReadings.get(cameraIdentifier) == null) {
					AprilTagResult tagResult = tagReadings.get(cameraIdentifier);
					String cameraIdentifier2 = poseReadings.keySet().toArray(new String[0])[0];
					if (poseReadings.get(cameraIdentifier2) == null) {
						AprilTagResult tagResult2 = tagReadings.get(cameraIdentifier2);
						double compare00 = tagResult.tagPose0.getTranslation().getDistance(tagResult2.tagPose0.getTranslation());
						double compare10 = tagResult.tagPose1.getTranslation().getDistance(tagResult2.tagPose0.getTranslation());
						double compare01 = tagResult.tagPose0.getTranslation().getDistance(tagResult2.tagPose1.getTranslation());
						double compare11 = tagResult.tagPose1.getTranslation().getDistance(tagResult2.tagPose1.getTranslation());

						if (compare00 > compare10 && compare00 > compare01 && compare00 > compare11) {
							poseReadings.put(cameraIdentifier, tagResult.tagPose0);
							poseReadings.put(cameraIdentifier2, tagResult.tagPose0);
						} else if (compare10 > compare01 && compare10 > compare11) {
							poseReadings.put(cameraIdentifier, tagResult.tagPose1);
							poseReadings.put(cameraIdentifier2, tagResult.tagPose0);
						} else if (compare01 > compare11) {
							poseReadings.put(cameraIdentifier, tagResult.tagPose0);
							poseReadings.put(cameraIdentifier2, tagResult.tagPose1);
						} else {
							poseReadings.put(cameraIdentifier, tagResult.tagPose1);
							poseReadings.put(cameraIdentifier2, tagResult.tagPose1);
						}
					} else {
						Pose3d tagPose2 = poseReadings.get(cameraIdentifier2);
						if (tagPose2.getTranslation().getDistance(tagResult.tagPose0.getTranslation()) <
						tagPose2.getTranslation().getDistance(tagResult.tagPose1.getTranslation())) {
							poseReadings.put(cameraIdentifier, tagResult.tagPose0);
						} else {
							poseReadings.put(cameraIdentifier, tagResult.tagPose1);
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
}

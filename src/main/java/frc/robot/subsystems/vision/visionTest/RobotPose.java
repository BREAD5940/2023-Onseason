// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.visionTest;

import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public class RobotPose {
	public Pose3d pose;
	public Pose3d poseAlt;

	public RobotPose(Pose3d pose, Pose3d poseAlt) {
		this.pose = pose;
		this.poseAlt = poseAlt;
	}
}

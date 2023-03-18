// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.visionTest;

import edu.wpi.first.math.geometry.Pose3d;
import static frc.robot.Constants.Vision.*;

/** Add your docs here. */
public class AprilTagResult {
    public double error0;
    public Pose3d tagPose0;
    public double error1;
    public Pose3d tagPose1;
    public Pose3d bestPose;
    public double timestamp;
    public boolean isOld = false;

    public AprilTagResult() {
    }

    public AprilTagResult(double error0, Pose3d tagPose0, double error1, Pose3d tagPose1, double timestamp) {
        this.error0 = error0;
        this.tagPose0 = tagPose0;
        this.error1 = error1;
        this.tagPose1 = tagPose1;
        this.timestamp = timestamp;

        bestPose = getBestPose();
    }

    public Pose3d getBestPose() {
        if (error0 < error1 * AMBIGUITY_THRESHOLD) {
            return tagPose0;
        } else if (error1 < error0 * AMBIGUITY_THRESHOLD) {
            return tagPose1;
        } else {
            return null;
        }
    }
}

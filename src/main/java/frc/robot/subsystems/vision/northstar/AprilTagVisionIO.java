// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision.northstar;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose3d;

public interface AprilTagVisionIO {
    public static class AprilTagVisionIOInputs implements LoggableInputs {
		public double[] timestamps = new double[] {};
		public int[][] tagIds = new int[][] {};
		public Pose3d[][] poses0 = new Pose3d[][] {};
		public double[][] errors0 = new double[][] {};
		public Pose3d[][] poses1 = new Pose3d[][] {};
		public double[][] errors1 = new double[][] {};
		public long fps = 0;

		@Override
		public void toLog(LogTable table) {
			table.put("Timestamps", timestamps);
			/*
			 * table.put("FrameCount", frames.length); for (int i = 0; i < frames.length;
			 * i++) { table.put("Frame/" + Integer.toString(i), frames[i]); }
			 */
			table.put("Fps", fps);
		}

		@Override
		public void fromLog(LogTable table) {
			timestamps = table.getDoubleArray("Timestamps", timestamps);
			/*
			 * int frameCount = (int) table.getInteger("FrameCount", 0); frames = new
			 * double[frameCount][]; for (int i = 0; i < frameCount; i++) { frames[i] =
			 * table.getDoubleArray("Frame/" + Integer.toString(i), new double[] {}); }
			 */
			fps = table.getInteger("Fps", fps);
		}
	}

    /**
     * @param inputs This will get filled with the data from the latest tag readings. See AprilTagVisionIO.AprilTagVisionIOInputs for more info.
     */
    public default void updateInputs(AprilTagVisionIOInputs inputs) {
    }

    /**
     * @return A string that can be used to identify the camera
     */
    public default String getIdentifier() {
        return "";
    }
}
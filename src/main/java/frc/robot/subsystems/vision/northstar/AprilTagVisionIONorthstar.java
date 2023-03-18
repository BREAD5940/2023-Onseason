// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.s

package frc.robot.subsystems.vision.northstar;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commons.Alert;
import frc.robot.commons.Alert.AlertType;

public class AprilTagVisionIONorthstar implements AprilTagVisionIO {
    private final DoubleArraySubscriber observationSubscriber;
    private final IntegerSubscriber fpsSubscriber;
    private String identifier;
    private static final double disconnectedTimeout = 1.5;
    private final Alert disconnectedAlert;
    private final Timer disconnectedTimer = new Timer();

    public AprilTagVisionIONorthstar(String identifier) {
        this.identifier = identifier;
        var northstarTable = NetworkTableInstance.getDefault().getTable(identifier);

        var outputTable = northstarTable.getSubTable("output");
        observationSubscriber = outputTable
                .getDoubleArrayTopic("observations")
                .subscribe(
                        new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);

        disconnectedAlert = new Alert("No data from \"" + identifier + "\"", AlertType.ERROR);
        disconnectedTimer.start();
    }

    public void updateInputs(AprilTagVisionIOInputs inputs) {
        var queue = observationSubscriber.readQueue();
        inputs.timestamps = new double[queue.length];
        inputs.tagIds = new int[queue.length][];
        inputs.poses0 = new Pose3d[queue.length][];
        inputs.errors0 = new double[queue.length][];
        inputs.poses1 = new Pose3d[queue.length][];
        inputs.errors1 = new double[queue.length][];
        for (int i = 0; i < queue.length; i++) {
            inputs.timestamps[i] = queue[i].timestamp / 1000000.0;
            double[] values = queue[i].value;
			inputs.tagIds[i] = new int[values.length];
			inputs.poses0[i] = new Pose3d[values.length];
			inputs.errors0[i] = new double[values.length];
			inputs.poses1[i] = new Pose3d[values.length];
			inputs.errors1[i] = new double[values.length];
            for (int j = 0; j < values.length; j += 15) {
                inputs.tagIds[i][j] = ((int) values[j]);
                inputs.poses0[i][j] = openCVPoseToWPILibPose(
                        VecBuilder.fill(values[j + 1], values[j + 2], values[j + 3]),
                        VecBuilder.fill(values[j + 4], values[j + 5], values[j + 6]));
                inputs.errors0[i][j] = values[j + 7];
                inputs.poses1[i][j] = openCVPoseToWPILibPose(
                        VecBuilder.fill(values[j + 8], values[j + 9], values[j + 10]),
                        VecBuilder.fill(values[j + 11], values[j + 12], values[j + 13]));
                inputs.errors1[i][j] = values[j + 14];
            }
        }
        inputs.fps = fpsSubscriber.get();

        // Update disconnected alert
        if (queue.length > 0) {
            disconnectedTimer.reset();
        }
        disconnectedAlert.set(disconnectedTimer.hasElapsed(disconnectedTimeout));

    }

    private static Pose3d openCVPoseToWPILibPose(Vector<N3> tvec, Vector<N3> rvec) {
        return new Pose3d(new Translation3d(tvec.get(2, 0), -tvec.get(0, 0), -tvec.get(1, 0)), new Rotation3d(
                VecBuilder.fill(rvec.get(2, 0), -rvec.get(0, 0), -rvec.get(1, 0)),
                Math.sqrt(Math.pow(rvec.get(0, 0), 2) + Math.pow(rvec.get(1, 0), 2) + Math.pow(rvec.get(2, 0), 2))));
    }

    public String getIdentifier() {
        return identifier;
    }
}
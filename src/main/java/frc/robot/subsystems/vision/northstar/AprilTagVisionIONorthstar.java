// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.s

package frc.robot.subsystems.vision.northstar;

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
        /*var queue = observationSubscriber.readQueue();
        inputs.timestamps = new double[queue.length];
        inputs.frames = new double[queue.length][];
        for (int i = 0; i < queue.length; i++) {
            inputs.timestamps[i] = queue[i].timestamp / 1000000.0;
            inputs.frames[i] = queue[i].value;
        }
        inputs.fps = fpsSubscriber.get();

        // Update disconnected alert
        if (queue.length > 0) {
            disconnectedTimer.reset();
        }
        disconnectedAlert.set(disconnectedTimer.hasElapsed(disconnectedTimeout));*/
    }

    public String getIdentifier() {
        return identifier;
    }
}
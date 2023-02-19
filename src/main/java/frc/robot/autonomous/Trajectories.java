package frc.robot.autonomous;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** File for storing all of our trajectories */
public class Trajectories {

    public static Trajectory generateTrajectory(boolean clampedCubic, List<Pose2d> points, double maxVel, double maxAccel, double startVel, double endVel, TrajectoryConstraint... constraints) {
        TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel);
        for (TrajectoryConstraint c : constraints) {
            config.addConstraint(c);
        }
        config.setStartVelocity(startVel);
        config.setEndVelocity(endVel);
        if (!clampedCubic) {
            return TrajectoryGenerator.generateTrajectory(points.stream().map(point -> new Pose2d(point.getTranslation(), point.getRotation())).collect(Collectors.toList()), config);
        } else {
            List<Translation2d> interiorPoints = new ArrayList<Translation2d>();
            for (int i=1; i<points.size()-1; i++) {
                interiorPoints.add(points.get(i).getTranslation());
            }
            return TrajectoryGenerator.generateTrajectory(points.get(0), interiorPoints, points.get(points.size()-1), config);
        }
    }
    
}
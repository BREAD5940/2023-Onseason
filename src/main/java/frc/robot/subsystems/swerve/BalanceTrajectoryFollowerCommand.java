// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.AllianceFlipUtil;

/** A command used to balence ontop of the charging staition */
public class BalanceTrajectoryFollowerCommand extends TrajectoryFollowerCommand {

	public BalanceTrajectoryFollowerCommand(PathPlannerTrajectory trajectory, Supplier<Rotation2d> startHeading,
			Swerve swerve, boolean stop) {
		super(trajectory, startHeading, swerve, stop);
	}

	public BalanceTrajectoryFollowerCommand(PathPlannerTrajectory trajectory, Swerve swerve, boolean stop) {
		super(trajectory, swerve, stop);
	}

	public void execute() {
		PathPlannerState goal = (PathPlannerState) trajectory.sample(timer.get());
		Trajectory.State wpilibGoal = AllianceFlipUtil.apply(goal);
		Rotation2d swerveRot;
		if (Robot.alliance == DriverStation.Alliance.Red) {
			swerveRot = new Rotation2d(
					-goal.holonomicRotation.getCos(),
					goal.holonomicRotation.getSin());
		} else {
			swerveRot = goal.holonomicRotation;
		}
		Pose2d currentRobotPose = RobotContainer.poseEstimator.getLatestPose();
		// checks of the robot is close enough on the y axie then to stop movement on
		// the y axie to improve auto balencing
		if (Math.abs(currentRobotPose.getY() - trajectory.getEndState().poseMeters.getY()) < Units.inchesToMeters(2) &&
				currentRobotPose.getTranslation().getDistance(trajectory.getEndState().poseMeters.getTranslation()) > 0.5) {
			wpilibGoal.poseMeters = new Pose2d(wpilibGoal.poseMeters.getX(), currentRobotPose.getY(),
					wpilibGoal.poseMeters.getRotation());
		}
		ChassisSpeeds adjustedSpeeds = autonomusController.calculate(RobotContainer.poseEstimator.getLatestPose(),
				wpilibGoal, swerveRot);
		swerve.requestVelocity(
				adjustedSpeeds, false, true);
		Logger.getInstance().recordOutput("Trajectory Goal",
				new Pose2d(wpilibGoal.poseMeters.getTranslation(), swerveRot));
	}
}

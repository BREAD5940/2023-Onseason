package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commons.BreadHolonomicDriveController;
import frc.robot.subsystems.Superstructure;

public class Drive2Point extends CommandBase {

    private PathPlannerTrajectory trajectory;
    private final Supplier<Pose2d> endPosition;
    private final Swerve swerve;
    private final double endVelocity;
    private final PathConstraints constraints;
    private final Timer timer = new Timer();
    private boolean failedToCreateTrajectory = false;
    private final Superstructure superstructure;

    public final BreadHolonomicDriveController autonomousController = new BreadHolonomicDriveController(
        new PIDController(8, 0, 0), 
        new PIDController(8, 0, 0), 
        new PIDController(2.0, 0, 0.1)
    );

    public Drive2Point(Supplier<Pose2d> endPosition, double endVelocity, PathConstraints constraints, Swerve swerve) {
        this.endPosition = endPosition;
        this.endVelocity = endVelocity;
        this.constraints = constraints;
        this.swerve = swerve;
    }
    
    @Override
    public void initialize() {
        try {
            Rotation2d currentHeading;

            if (swerve.getVelocity().getNorm() < 0.2) {
                currentHeading = endPosition.get().getRotation();
            } else {
                currentHeading = swerve.getVelocity().rotateBy(swerve.getRotation2d()).getAngle();
            }
            trajectory = PathPlanner.generatePath(
                constraints, 
                new PathPoint(swerve.getPose().getTranslation(), currentHeading, swerve.getPose().getRotation(), swerve.getVelocity().getNorm()),
                new PathPoint(endPosition.get().getTranslation(), endPosition.get().getRotation(), endPosition.get().getRotation())
            );
            Logger.getInstance().recordOutput("OnTheFlyTrajectoryGeneration", trajectory);
        } catch (Exception e) {
            System.out.println("Failed to create on-the-fly trajectory!");
            failedToCreateTrajectory = true;
        }
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        PathPlannerState goal = (PathPlannerState) trajectory.sample(timer.get());
        ChassisSpeeds adjustedSpeeds = autonomousController.calculate(swerve.getPose(), goal, goal.holonomicRotation); 
        Logger.getInstance().recordOutput("TrajectoryVXMetersPerSecond", adjustedSpeeds.vxMetersPerSecond);
        Logger.getInstance().recordOutput("TrajectoryVYMetersPerSecond", adjustedSpeeds.vyMetersPerSecond);
        Logger.getInstance().recordOutput("TrajectoryxError", autonomousController.m_poseError.getX());
        Logger.getInstance().recordOutput("TrajectoryYError", autonomousController.m_poseError.getY());
        Logger.getInstance().recordOutput("TrajectoryThetaError", autonomousController.m_poseError.getRotation().getDegrees());
        System.out.printf("%f.2 %f.2 %f.2\n", adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond);
        swerve.requestVelocity(adjustedSpeeds, false);

        if (timer.get() >= trajectory.getTotalTimeSeconds() - 1.5) {
            superstructure.requestIntakeConeDoubleSubstation();
        }
    }

    @Override
    public boolean isFinished() {
        return failedToCreateTrajectory;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.requestVelocity(new ChassisSpeeds(0.0, 0.0, 0.0), false);
    }

}

package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
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
    private final Superstructure superstructure;
    private final Timer timer = new Timer();
    private boolean failedToCreateTrajectory = false;
    public final BreadHolonomicDriveController autonomousController = new BreadHolonomicDriveController(
        new PIDController(8, 0, 0), 
        new PIDController(8, 0, 0), 
        new PIDController(2.0, 0, 0.1)
    );

    public Drive2Point(Supplier<Pose2d> endPosition, double endVelocity, PathConstraints constraints, Swerve swerve, Superstructure superstructure) {
        this.endPosition = endPosition;
        this.endVelocity = endVelocity;
        this.constraints = constraints;
        this.swerve = swerve;
        this.superstructure = superstructure;
    }
    
    @Override
    public void initialize() {
        try {
            trajectory = PathPlanner.generatePath(
                constraints, 
                new PathPoint(swerve.getPose().getTranslation(), swerve.getPose().getRotation(), new Rotation2d(), swerve.getVelocity().getNorm()),
                new PathPoint(endPosition.get().getTranslation(), endPosition.get().getRotation(), new Rotation2d(), endVelocity)
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
        Trajectory.State goal = trajectory.sample(timer.get());
        ChassisSpeeds adjustedSpeeds = autonomousController.calculate(swerve.getPose(), goal, goal.poseMeters.getRotation()); 
        Logger.getInstance().recordOutput("TrajectoryVXMetersPerSecond", adjustedSpeeds.vxMetersPerSecond);
        Logger.getInstance().recordOutput("TrajectoryVYMetersPerSecond", adjustedSpeeds.vyMetersPerSecond);
        Logger.getInstance().recordOutput("TrajectoryxError", autonomousController.m_poseError.getX());
        Logger.getInstance().recordOutput("TrajectoryYError", autonomousController.m_poseError.getY());
        Logger.getInstance().recordOutput("TrajectoryThetaError", autonomousController.m_poseError.getRotation().getDegrees());
        System.out.printf("%f.2 %f.2 %f.2\n", adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond);
        swerve.requestVelocity(adjustedSpeeds, false);
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

package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commons.BreadHolonomicDriveController;

public class TrajectoryFollowerCommand extends CommandBase {

    private final PathPlannerTrajectory trajectory;
    private final Supplier<Rotation2d> startHeading;
    private final Swerve swerve;
    private final Timer timer = new Timer();
    private final boolean stop;
    public final BreadHolonomicDriveController autonomusController = new BreadHolonomicDriveController(
        new PIDController(8.0, 0, 0), 
        new PIDController(8.0, 0, 0), 
        new PIDController(4.0, 0, 0)
    );

    public TrajectoryFollowerCommand(PathPlannerTrajectory trajectory, Supplier<Rotation2d> startHeading, Swerve swerve, boolean stop) {
        this.trajectory = trajectory;
        this.startHeading = startHeading;
        this.swerve = swerve;
        this.stop = stop;
        addRequirements(swerve);
    }

    public TrajectoryFollowerCommand(PathPlannerTrajectory trajectory, Swerve swerve, boolean stop) {
        this(trajectory, null, swerve, stop);
    }

    @Override
    public void initialize() {
        if (startHeading != null) {
            swerve.reset(new Pose2d(
                trajectory.sample(0.0).poseMeters.getTranslation(), 
                startHeading.get()
            ));
        }
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        PathPlannerState goal = (PathPlannerState) trajectory.sample(timer.get());
        ChassisSpeeds adjustedSpeeds = autonomusController.calculate(swerve.getPose(), goal, goal.holonomicRotation); 
        swerve.requestVelocity(
            adjustedSpeeds, false
        );
        Logger.getInstance().recordOutput("Trajectory Goal X", goal.poseMeters.getX());
        Logger.getInstance().recordOutput("Trajectory Goal Y", goal.poseMeters.getY());
        Logger.getInstance().recordOutput("Trajectory Goal Heading", goal.poseMeters.getRotation().getDegrees());
    }

    @Override
    public boolean isFinished() {
        if (stop) {
            return timer.get() >= trajectory.getTotalTimeSeconds();
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) { 
        swerve.requestPercent(new ChassisSpeeds(0, 0, 0), false);
    }

}
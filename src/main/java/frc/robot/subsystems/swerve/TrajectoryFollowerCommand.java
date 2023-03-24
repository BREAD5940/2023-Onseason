package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.AllianceFlipUtil;
import frc.robot.commons.BreadHolonomicDriveController;

public class TrajectoryFollowerCommand extends CommandBase {

    private final PathPlannerTrajectory trajectory;
    private final Supplier<Rotation2d> startHeading;
    private final Swerve swerve;
    private final Timer timer = new Timer();
    private final Timer balanceTimer = new Timer();
    private boolean balanceStarted = false;
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
            Pose2d startPose = AllianceFlipUtil.apply(new Pose2d(
                trajectory.sample(0.0).poseMeters.getTranslation(), 
                startHeading.get()));
            RobotContainer.poseEstimator.resetPose(startPose);
        }
        timer.reset();
        timer.start();
        balanceTimer.reset();
        balanceTimer.stop();
        balanceStarted = false;
    }

    @Override
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
        ChassisSpeeds adjustedSpeeds = autonomusController.calculate(RobotContainer.poseEstimator.getLatestPose(), wpilibGoal, swerveRot); 
        swerve.requestVelocity(
            adjustedSpeeds, false, true
        );
        Logger.getInstance().recordOutput("Trajectory Goal", new Pose2d(wpilibGoal.poseMeters.getTranslation(), swerveRot));
    }

    @Override
    public boolean isFinished() {
        if (stop) {
            return timer.get() >= trajectory.getTotalTimeSeconds();
        } else {
            Pose2d poseError = trajectory.getEndState().poseMeters.relativeTo(RobotContainer.poseEstimator.getLatestPose());
            if (timer.get() >= trajectory.getTotalTimeSeconds() && Math.abs(swerve.getPitch()) < 0.15 && !balanceStarted && poseError.getTranslation().getNorm() < 0.07) {
                balanceStarted  = true;
                balanceTimer.start();
            } 

            if (timer.get() >= trajectory.getTotalTimeSeconds() && Math.abs(swerve.getPitch()) > 0.15) {
                balanceStarted = false;
                balanceTimer.stop();
                balanceTimer.reset();
            }

            return timer.get() >= trajectory.getTotalTimeSeconds() && balanceTimer.get() > 2.0;
        }
    }

    @Override
    public void end(boolean interrupted) { 
        swerve.requestPercent(new ChassisSpeeds(0, 0, 0), false);
    }

}
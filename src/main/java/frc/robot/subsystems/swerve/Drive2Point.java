package frc.robot.subsystems.swerve;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.autonomous.Trajectories;
import frc.robot.commons.BreadHolonomicDriveController;

public class Drive2Point extends CommandBase {

    private Trajectory trajectory;
    private final Supplier<Pose2d> endPosition;
    private final Swerve swerve;
    private final double endVelocity;
    private final double maxVelocity;
    private final double maxAccel;
    private final Timer timer = new Timer();
    private boolean failedToCreateTrajectory = false;

    public final BreadHolonomicDriveController autonomousController = new BreadHolonomicDriveController(
        new PIDController(0, 0, 0), 
        new PIDController(0, 0, 0), 
        new PIDController(0, 0, 0)
    );

    public Drive2Point(Supplier<Pose2d> endPosition, double endVelocity, double maxVelocity, double maxAccel, Swerve swerve) {
        this.endPosition = endPosition;
        this.endVelocity = endVelocity;
        this.maxVelocity = maxVelocity;
        this.maxAccel = maxAccel;
        this.swerve = swerve;
    }
    
    @Override
    public void initialize() {
        failedToCreateTrajectory = false;
        try {
            timer.reset();
            timer.start();
            Rotation2d currentHeading;
            if (swerve.getVelocity().getNorm() < 0.2) {
                currentHeading = Rotation2d.fromDegrees(180.0);
            } else {
                currentHeading = swerve.getVelocity().rotateBy(swerve.getRotation2d().rotateBy(Rotation2d.fromDegrees(180.0))).getAngle();
            }                
            trajectory = Trajectories.generateTrajectory(true, List.of(
                new Pose2d(RobotContainer.poseEstimator.getLatestPose().getTranslation(), currentHeading),
                endPosition.get()
            ), maxVelocity, maxAccel, swerve.getVelocity().getNorm(), endVelocity);
            Logger.getInstance().recordOutput("OnTheFlyTrajectoryGeneration", trajectory);
        } catch (Exception e) {
            failedToCreateTrajectory = true;
        }
    }

    @Override
    public void execute() {
        Trajectory.State goal = trajectory.sample(timer.get());
        ChassisSpeeds adjustedSpeeds = autonomousController.calculate(RobotContainer.poseEstimator.getLatestPose(), goal, Rotation2d.fromDegrees(0.0)); 
        Logger.getInstance().recordOutput("AdjustedTrajectoryVXMetersPerSecond", adjustedSpeeds.vxMetersPerSecond);
        Logger.getInstance().recordOutput("AdjustedTrajectoryVYMetersPerSecond", adjustedSpeeds.vyMetersPerSecond);
        Logger.getInstance().recordOutput("TrajectoryGoalMetersPerSecond", goal.velocityMetersPerSecond);
        Logger.getInstance().recordOutput("TrajectoryxError", autonomousController.m_poseError.getX());
        Logger.getInstance().recordOutput("TrajectoryYError", autonomousController.m_poseError.getY());
        Logger.getInstance().recordOutput("TrajectoryThetaError", autonomousController.m_poseError.getRotation().getDegrees());
        System.out.printf("%f.2 %f.2 %f.2\n", adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond);
        swerve.requestVelocity(adjustedSpeeds, false, true);
    }

    @Override
    public boolean isFinished() {
        return failedToCreateTrajectory;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.requestVelocity(new ChassisSpeeds(0.0, 0.0, 0.0), 
        false, false);
    }

}

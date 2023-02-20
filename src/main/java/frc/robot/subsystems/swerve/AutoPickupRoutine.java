package frc.robot.subsystems.swerve;

import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.FieldConstants.LoadingZone;
import frc.robot.autonomous.Trajectories;
import frc.robot.commons.BreadHolonomicDriveController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.endeffector.EndEffector;

import static frc.robot.Constants.Elevator.*;

public class AutoPickupRoutine extends CommandBase {

    private Trajectory trajectory;
    private final BiFunction<Pose2d, Double, Rotation2d> refHeading;
    private final Supplier<Pose2d> endPosition;
    private final Swerve swerve;
    private final Superstructure superstructure;
    private final Timer timer = new Timer();
    private boolean failedToCreateTrajectory = false;
    private boolean deployed = false;
    public final BreadHolonomicDriveController autonomusController = new BreadHolonomicDriveController(
        new PIDController(4.0, 0, 0), 
        new PIDController(4.0, 0, 0), 
        new PIDController(4.0, 0, 0.1)
    );

    public AutoPickupRoutine(Supplier<Pose2d> endPosition, BiFunction<Pose2d, Double, Rotation2d> refHeading, Swerve swerve, Superstructure superstructure) {
        this.endPosition = endPosition;
        this.refHeading = refHeading;
        this.swerve = swerve;
        this.superstructure = superstructure;
        addRequirements(swerve, superstructure);
    }

    @Override
    public void initialize() {
        failedToCreateTrajectory = false;
        deployed = false;
        try {
            timer.reset();
            timer.start();
            // Rotation2d currentHeading;
            // if (swerve.getVelocity().getNorm() < 0.2) {
            //     if (swerve.getPose().getTranslation().getX() < endPosition.get().getX() - Units.feetToMeters(10.0)) {
            //         currentHeading = new Translation2d(endPosition.get().getX() - Units.feetToMeters(5.0), endPosition.get().getY()).minus(swerve.getPose().getTranslation()).getAngle();
            //     } else {
            //         currentHeading = Rotation2d.fromDegrees(0.0);
            //     }
            // } else {
            //     System.out.println("Adjusting path by swerve velocity!\n\n\n\n");
            //     currentHeading = swerve.getVelocity().rotateBy(swerve.getRotation2d().rotateBy(Rotation2d.fromDegrees(180.0))).getAngle();
            // }                
            Rotation2d currentHeading = new Rotation2d();
            trajectory = Trajectories.generateTrajectory(false, List.of(
                new Pose2d(RobotContainer.poseEstimator.getLatestPose().getTranslation(), currentHeading),
                // new Pose2d(endPosition.get().getX() - Units.feetToMeters(4-.0), endPosition.get().getY(), endPosition.get().getRotation()),
                new Pose2d(endPosition.get().getX(), endPosition.get().getY(), endPosition.get().getRotation())
            ), 1.5, 1.0, swerve.getVelocity().getNorm(), 0.0);
            Logger.getInstance().recordOutput("OnTheFlyTrajectoryGeneration", trajectory);
        } catch (Exception e) {
            System.out.println("Failed to create on-the-fly trajectory!");
            failedToCreateTrajectory = true;
        }
        superstructure.requestIntakeConeDoubleSubstation();
    }

    @Override
    public void execute() {
        Trajectory.State goal = trajectory.sample(timer.get());
        ChassisSpeeds adjustedSpeeds = autonomusController.calculate(RobotContainer.poseEstimator.getLatestPose(), goal, refHeading.apply(RobotContainer.poseEstimator.getLatestPose(), timer.get())); 
        Logger.getInstance().recordOutput("AdjustedTrajectoryVXMetersPerSecond", adjustedSpeeds.vxMetersPerSecond);
        Logger.getInstance().recordOutput("AdjustedTrajectoryVYMetersPerSecond", adjustedSpeeds.vyMetersPerSecond);
        Logger.getInstance().recordOutput("TrajectoryGoalMetersPerSecond", goal.velocityMetersPerSecond);
        Logger.getInstance().recordOutput("TrajectoryxError", autonomusController.m_poseError.getX());
        Logger.getInstance().recordOutput("TrajectoryYError", autonomusController.m_poseError.getY());
        Logger.getInstance().recordOutput("TrajectoryThetaError", autonomusController.m_poseError.getRotation().getDegrees());
        System.out.printf("%f.2 %f.2 %f.2\n", adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond);
        swerve.requestVelocity(adjustedSpeeds, false);
    }

    @Override
    public boolean isFinished() { 
        return failedToCreateTrajectory;
    }

    @Override
    public void end(boolean interrupted) { 
        swerve.requestVelocity(new ChassisSpeeds(0, 0, 0), false);
    }
    
}
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
import frc.robot.commons.AllianceFlipUtil;
import frc.robot.commons.BreadHolonomicDriveController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.endeffector.EndEffector;

import static frc.robot.Constants.Elevator.*;

public class AutoPickupRoutine extends CommandBase {

    private Trajectory trajectory;
    private final BiFunction<Pose2d, Double, Rotation2d> refHeading;
    private boolean deployedElevator = false;
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
        RobotContainer.northstarVision.mStdDevScalar = 0.5;
        failedToCreateTrajectory = false;
        deployed = false;
        deployedElevator = false;
        try {
            timer.reset();
            timer.start();
            Pose2d start = AllianceFlipUtil.apply(new Pose2d(RobotContainer.poseEstimator.getLatestPose().getTranslation(), new Rotation2d()));        
            Pose2d end = AllianceFlipUtil.apply(new Pose2d(endPosition.get().getX(), endPosition.get().getY(), endPosition.get().getRotation()));            
    
            trajectory = Trajectories.generateTrajectory(false, List.of(
                start, 
                end
            ), 1.5, 1.0, swerve.getVelocity().getNorm(), 0.0);
            Logger.getInstance().recordOutput("OnTheFlyTrajectoryGeneration", trajectory);
        } catch (Exception e) {
            System.out.println("Failed to create on-the-fly trajectory!");
            failedToCreateTrajectory = true;
        }
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
        swerve.requestVelocity(adjustedSpeeds, false, false);

        if (trajectory.getTotalTimeSeconds() - timer.get() <= 2.5 && !deployedElevator) {
            deployedElevator = true;
            superstructure.requestIntakeConeDoubleSubstation();
        }
    }

    @Override
    public boolean isFinished() { 
        return failedToCreateTrajectory;
    }

    @Override
    public void end(boolean interrupted) { 
        RobotContainer.northstarVision.mStdDevScalar = 2.0;
        swerve.requestVelocity(new ChassisSpeeds(0, 0, 0), false, false);
    }
    
}
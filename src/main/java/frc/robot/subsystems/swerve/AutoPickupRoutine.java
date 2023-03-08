package frc.robot.subsystems.swerve;

import java.util.List;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.FieldConstants.LoadingZone;
import frc.robot.autonomous.Trajectories;
import frc.robot.commons.AllianceFlipUtil;
import frc.robot.commons.BreadHolonomicDriveController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.endeffector.EndEffector;

import static frc.robot.Constants.Elevator.*;
import static frc.robot.Constants.RobotLocations.*;

public class AutoPickupRoutine extends CommandBase {

    private Trajectory trajectory;
    private boolean deployedElevator = false;
    private final Swerve swerve;
    private final Superstructure superstructure;
    private final Timer timer = new Timer();
    private boolean failedToCreateTrajectory = false;
    private BooleanSupplier closeDoubleSup;
    private BooleanSupplier farDoubleSup;
    private Pose2d end;
    public final BreadHolonomicDriveController autonomusController = new BreadHolonomicDriveController(
        new PIDController(4.0, 0, 0), 
        new PIDController(4.0, 0, 0), 
        new PIDController(2.0, 0, 0)
    );

    public AutoPickupRoutine(BooleanSupplier closeDoubleSup, BooleanSupplier farDoubleSup, Swerve swerve, Superstructure superstructure) {
        this.swerve = swerve;
        this.superstructure = superstructure;
        this.closeDoubleSup = closeDoubleSup; 
        this.farDoubleSup = farDoubleSup;
        addRequirements(swerve, superstructure);
    }

    @Override
    public void initialize() {
        failedToCreateTrajectory = false;
        deployedElevator = false;

        Pose2d closeEndPose = AllianceFlipUtil.apply(CLOSE_PICKUP_LOCATION);
        Pose2d farEndPose = AllianceFlipUtil.apply(FAR_PICKUP_LOCATION);
        if (DriverStation.getAlliance() == Alliance.Blue) {

        } else {
            
        }

        try {
            timer.reset();
            timer.start();
            Pose2d start = AllianceFlipUtil.apply(new Pose2d(RobotContainer.poseEstimator.getLatestPose().getTranslation(), new Rotation2d()));        
            end = AllianceFlipUtil.apply(new Pose2d(closeEndPose.getX(), closeEndPose.getY(), closeEndPose.getRotation()));            
    
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
        ChassisSpeeds adjustedSpeeds = autonomusController.calculate(RobotContainer.poseEstimator.getLatestPose(), goal, new Rotation2d()); 
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
        swerve.requestVelocity(new ChassisSpeeds(0, 0, 0), false, false);
    }
    
}
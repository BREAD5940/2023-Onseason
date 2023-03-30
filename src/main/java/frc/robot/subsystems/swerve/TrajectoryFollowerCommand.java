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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.AllianceFlipUtil;
import frc.robot.commons.BreadHolonomicDriveController;
import frc.robot.commons.LoggedTunableNumber;
import frc.robot.subsystems.vision.northstar.AprilTagVision;

public class TrajectoryFollowerCommand extends CommandBase {

    private final PathPlannerTrajectory trajectory;
    private final Supplier<Rotation2d> startHeading;
    private final Swerve swerve;
    private final Timer timer = new Timer();
    private final Timer balanceTimer = new Timer();
    private boolean balanceStarted = false;
    private final boolean dontBalanceAtEnd;
    private final LoggedTunableNumber xControllerP = new LoggedTunableNumber("BalanceTunes/xControllerP", 8.0);
    private final LoggedTunableNumber xControllerD = new LoggedTunableNumber("BalanceTunes/xControllerD", 0.0);
    private final LoggedTunableNumber yControllerP = new LoggedTunableNumber("BalanceTunes/yControllerP", 8.0);
    private final LoggedTunableNumber yControllerD = new LoggedTunableNumber("BalanceTunes/yControllerD", 0.0);
    private final LoggedTunableNumber thetaControllerP = new LoggedTunableNumber("BalanceTunes/thetaControllerP", 4.0);
    private final LoggedTunableNumber thetaControllerD = new LoggedTunableNumber("BalanceTunes/thetaControllerD", 0.0);

    private double X_CONTROLLER_P = 8.0;
    private double Y_CONTROLLER_P = 8.0;
    private double THETA_CONTROLLER_P = 4.0;
    private double X_CONTROLLER_D = 0.0;
    private double Y_CONTROLLER_D = 0.0;
    private double THETA_CONTROLLER_D = 0.0;

    public final BreadHolonomicDriveController autonomusController = new BreadHolonomicDriveController(
        new PIDController(X_CONTROLLER_P, 0, X_CONTROLLER_D), 
        new PIDController(Y_CONTROLLER_P, 0, Y_CONTROLLER_D), 
        new PIDController(THETA_CONTROLLER_P, 0, THETA_CONTROLLER_D)
    );

    public TrajectoryFollowerCommand(PathPlannerTrajectory trajectory, Supplier<Rotation2d> startHeading, Swerve swerve, boolean dontBalanceAtEnd) {
        this.trajectory = trajectory;
        this.startHeading = startHeading;
        this.swerve = swerve;
        this.dontBalanceAtEnd = dontBalanceAtEnd;
        addRequirements(swerve);
    }

    public TrajectoryFollowerCommand(PathPlannerTrajectory trajectory, Swerve swerve, boolean dontBalanceAtEnd) {
        this(trajectory, null, swerve, dontBalanceAtEnd);
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
        Pose2d poseError = wpilibGoal.poseMeters.relativeTo(RobotContainer.poseEstimator.getLatestPose());
        Pose2d loggedGoal = new Pose2d(wpilibGoal.poseMeters.getTranslation(), swerveRot);
        if (!dontBalanceAtEnd && poseError.getTranslation().getNorm() < Units.inchesToMeters(18.0)) {
            autonomusController.setXController_P(xControllerP.get());
            autonomusController.setXController_D(xControllerD.get());
            autonomusController.setYController_P(yControllerP.get());
            autonomusController.setYController_D(yControllerD.get());      
            autonomusController.setThetaController_P(thetaControllerP.get());
            autonomusController.setThetaController_D(thetaControllerD.get());
            Logger.getInstance().recordOutput("TrajectoryFollowerController/UsingBalancePIDs", true);
        } else {
            autonomusController.setXController_P(X_CONTROLLER_P);
            autonomusController.setXController_D(X_CONTROLLER_D);
            autonomusController.setYController_P(Y_CONTROLLER_P);
            autonomusController.setYController_D(Y_CONTROLLER_D);
            autonomusController.setThetaController_P(THETA_CONTROLLER_P);
            autonomusController.setThetaController_D(THETA_CONTROLLER_D);
            Logger.getInstance().recordOutput("TrajectoryFollowerController/UsingBalancePIDs", false);
        }
        ChassisSpeeds adjustedSpeeds = autonomusController.calculate(RobotContainer.poseEstimator.getLatestPose(), wpilibGoal, swerveRot); 
        swerve.requestVelocity(
            adjustedSpeeds, false, true
        );
        
        Logger.getInstance().recordOutput("TrajectoryFollowerController/TrajectoryGoal", loggedGoal);
    }

    @Override
    public boolean isFinished() {
        PathPlannerState goal = (PathPlannerState) trajectory.sample(timer.get());
        Trajectory.State wpilibGoal = AllianceFlipUtil.apply(goal);
        if (dontBalanceAtEnd) {
            return timer.get() >= trajectory.getTotalTimeSeconds();
        } else {
            Pose2d poseError = wpilibGoal.poseMeters.relativeTo(RobotContainer.poseEstimator.getLatestPose());
            if (timer.get() >= trajectory.getTotalTimeSeconds() && Math.abs(swerve.getPitch()) < Units.degreesToRadians(4.0) && !balanceStarted && poseError.getTranslation().getNorm() < 0.07) {
                balanceStarted  = true;
                balanceTimer.start();
            } 

            if (timer.get() >= trajectory.getTotalTimeSeconds() && Math.abs(swerve.getPitch()) > Units.degreesToRadians(4.0)) {
                balanceStarted = false;
                balanceTimer.stop();
                balanceTimer.reset();
            }

            return timer.get() >= trajectory.getTotalTimeSeconds() && balanceTimer.get() > 0.25;
        }
    }

    @Override
    public void end(boolean interrupted) { 
        swerve.requestPercent(new ChassisSpeeds(0, 0, 0), false);
    }

}
package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.AllianceFlipUtil;
import frc.robot.commons.BreadUtil;
import static frc.robot.Constants.Drive.*;
import static frc.robot.FieldConstants.*;

import org.littletonrobotics.junction.Logger;

public class AlignChargeStationCommand extends CommandBase {
  private final Swerve swerve;

  private final PIDController xController = new PIDController(4.0, 0.0, 0.004);
  private final PIDController yController = new PIDController(4.0, 0.0, 0.004);
  private final PIDController thetaController = new PIDController(5.0, 0.0, 0.0);

  private Pose2d targetPose;

  private final double X_ERROR_TOLERANCE = Units.inchesToMeters(2.0);
  private final double Y_ERROR_TOLERANCE = Units.inchesToMeters(2.0);
  private final double THETA_ERROR_TOLERANCE = Units.degreesToRadians(1.0);

  public AlignChargeStationCommand(Swerve swerve) {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // TOOD: define target pose
    targetPose = AllianceFlipUtil.apply(new Pose2d(new Translation2d(fieldLength - 12.764470036936714, 5.24633), Rotation2d.fromDegrees(180.0)));
    Swerve.alignedChargeStation = false;
    Swerve.aligningChargeStation = true;

    // Reset PID controllers
    xController.reset();
    yController.reset();
    thetaController.reset();
  }

  @Override
  public void execute() {
    Pose2d currentRobotPose = RobotContainer.poseEstimator.getLatestPose();
    Pose2d poseError = targetPose.relativeTo(currentRobotPose);

    // Calculate the outputs for the PID controllers
    double xOutput = xController.calculate(currentRobotPose.getX(), targetPose.getX());
    double yOutput = yController.calculate(currentRobotPose.getY(), targetPose.getY());
    double thetaOutput = thetaController.calculate(
      currentRobotPose.getRotation().getRadians(), targetPose.getRotation().getRadians()
    );

    xOutput = MathUtil.clamp(xOutput, -1, 1);
    yOutput = MathUtil.clamp(yOutput, -1, 1);
    thetaOutput = MathUtil.clamp(thetaOutput, -1.0, 1.0);

    if (!Swerve.alignedChargeStation) {
       // Aligns to the correct pose based on the PID outputs
      swerve.requestVelocity(
        new ChassisSpeeds(xOutput, yOutput, thetaOutput),
        true,
        false
      );

      // Check if aligned with the target pose
      if (
        Math.abs(poseError.getX()) < X_ERROR_TOLERANCE &&
        Math.abs(poseError.getY()) < Y_ERROR_TOLERANCE &&
        Math.abs(poseError.getRotation().getDegrees()) < THETA_ERROR_TOLERANCE
      ) {
        Swerve.alignedChargeStation = true;
      }
    } else {
      double y = BreadUtil.deadband(RobotContainer.driver.getRightX(), 0.1);
      double x = BreadUtil.deadband(RobotContainer.driver.getRightY(), 0.1);
      double scale = 0.25;
      double dy;
      double dx;

      if (Robot.alliance == DriverStation.Alliance.Blue) {
        dx = Math.pow(-x, 1) * scale;
        dy = Math.pow(-y, 1) * scale;
        
      } else {
        dx = Math.pow(-x, 1) * scale * -1;
        dy = Math.pow(-y, 1) * scale * -1;
      }

      // Lock x and heading, allowing driver input on y
      swerve.requestVelocity(
        new ChassisSpeeds(dx * ROBOT_MAX_VELOCITY, dy * ROBOT_MAX_VELOCITY, thetaOutput),
        true,
        false
      );
    }

    Logger.getInstance().recordOutput("AutoAlignChargeStation/TargetPose", targetPose);
    Logger.getInstance().recordOutput("AutoAlignChargeStation/TargetPoseError", poseError);
    Logger.getInstance().recordOutput("AutoAlignChargeStation/Aligned", Swerve.alignedChargeStation);
  }

  @Override
  public void end(boolean interrupted) {
    Swerve.aligningChargeStation = false;
    swerve.requestVelocity(new ChassisSpeeds(0.0, 0.0, 0.0), false, false);
  }

  /* Returns the alignment status */
  public boolean isAligned() {
    return Swerve.alignedChargeStation;
  }
}

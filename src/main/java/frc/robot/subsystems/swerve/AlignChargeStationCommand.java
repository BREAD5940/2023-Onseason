package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commons.AllianceFlipUtil;
import frc.robot.commons.BreadUtil;
import static frc.robot.Constants.Drive.*;

public class AlignChargeStationCommand extends CommandBase {
  private final Swerve swerve;

  private final PIDController xController = new PIDController(4.0, 0.0, 0.004);
  private final PIDController yController = new PIDController(4.0, 0.0, 0.004);
  private final PIDController thetaController = new PIDController(5.0, 0.0, 0.0);

  private Pose2d targetPose;
  private boolean aligned;

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
    targetPose = AllianceFlipUtil.apply(new Pose2d());
    aligned = false;

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
    double xOutput = xController.calculate(poseError.getX());
    double yOutput = yController.calculate(poseError.getY());
    double thetaOutput = thetaController.calculate(
      poseError.getRotation().getRadians()
    );

    if (!aligned) {
       // Aligns to the correct pose based on the PID outpus
      swerve.requestVelocity(
        new ChassisSpeeds(xOutput, yOutput, thetaOutput),
        true,
        false
      );

      // Check if aligned with the target pose
      if (
        Math.abs(xController.getPositionError()) < X_ERROR_TOLERANCE &&
        Math.abs(yController.getPositionError()) < Y_ERROR_TOLERANCE &&
        Math.abs(thetaController.getPositionError()) < THETA_ERROR_TOLERANCE
      ) {
        aligned = true;
      }
    } else {
      double y = BreadUtil.deadband(RobotContainer.driver.getRightX(), 0.1);
      double scale = RobotContainer.driver.getLeftBumper() ? 0.25 : 1.0;
      double dy;

      if (Robot.alliance == DriverStation.Alliance.Blue) {
        dy = Math.pow(-y, 1) * scale;
      } else {
        dy = Math.pow(-y, 1) * scale * -1;
      }

      // Lock x and heading, allowing driver input on y
      swerve.requestVelocity(
        new ChassisSpeeds(xOutput, dy * ROBOT_MAX_VELOCITY, thetaOutput),
        true,
        false
      );
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.requestVelocity(new ChassisSpeeds(0.0, 0.0, 0.0), false, false);
  }

  /* Returns the alignment status */
  public boolean isAligned() {
    return aligned;
  }
}

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class AlignChargeStationCommand extends CommandBase {
  private final Swerve swerve;

  private final PIDController xController = new PIDController(0, 0, 0);
  private final PIDController yController = new PIDController(0, 0, 0);
  private final PIDController thetaController = new PIDController(0, 0, 0);

  // TODO: Define target poses for each alliance
  private final Pose2d redTargetPose = new Pose2d();
  private final Pose2d blueTargetPose = new Pose2d();
  private Pose2d targetPose;

  private boolean aligned = false;

  public AlignChargeStationCommand(Swerve swerve) {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    targetPose =
      Robot.alliance == Alliance.Red ? redTargetPose : blueTargetPose;

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
        Math.abs(xController.getPositionError()) < Units.inchesToMeters(2.0) &&
        Math.abs(yController.getPositionError()) < Units.inchesToMeters(2.0) &&
        Math.abs(thetaController.getPositionError()) <
        Units.degreesToRadians(1.0)
      ) {
        aligned = true;
      }
    } else {
      // Lock x and heading, allowing driver input on y
      swerve.requestVelocity(
        new ChassisSpeeds(xOutput, RobotContainer.dy * 4.95, thetaOutput),
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

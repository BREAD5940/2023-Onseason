package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;

public class DoubleSubstationDriveAssistCommand extends CommandBase {

    private final Swerve swerve;
    private final Superstructure superstructure;
    private final PIDController thetaController = new PIDController(2.0, 0.0, 0.0);
    private Rotation2d headingGoal;
    private boolean deployed = false;

    public DoubleSubstationDriveAssistCommand(Swerve swerve, Superstructure superstructure) {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.swerve = swerve;
        this.superstructure = superstructure;
        addRequirements(swerve, superstructure);
    }

    @Override
    public void initialize() {
        headingGoal = DriverStation.getAlliance() == Alliance.Blue ? new Rotation2d() : new Rotation2d(Math.PI);
        deployed = false;
    }

    @Override
    public void execute() {
        Pose2d measurement = RobotContainer.poseEstimator.getLatestPose();
        double x = RobotContainer.driver.getLeftY();
        double y = RobotContainer.driver.getLeftX();
        double dx;
        double dy;
        if (Robot.alliance == DriverStation.Alliance.Blue) {
            dx = Math.abs(x) > 0.05 ? Math.pow(-x, 1) : 0.0;
            dy = Math.abs(y) > 0.05 ? Math.pow(-y, 1) : 0.0;
        } else {
            dx = Math.abs(x) > 0.05 ? Math.pow(-x, 1) * -1 : 0.0;
            dy = Math.abs(y) > 0.05 ? Math.pow(-y, 1) * -1 : 0.0;
        }
        double thetaFeedback = thetaController.calculate(
            measurement.getRotation().getRadians(),
            headingGoal.getRadians());
        thetaFeedback = MathUtil.clamp(thetaFeedback, -5.0, 5.0);
        Rotation2d error = headingGoal.minus(measurement.getRotation());
        if (!deployed && Math.abs(error.getDegrees()) < 2.0) {
            deployed = true;
            superstructure.requestIntakeConeDoubleSubstation();
        }

        swerve.requestVelocity(new ChassisSpeeds(dx, dy, thetaFeedback), true, false);
    }
    
    @Override
    public void end(boolean interrupted) { 
        swerve.requestVelocity(new ChassisSpeeds(0, 0, 0), false, false);
    }
    
}

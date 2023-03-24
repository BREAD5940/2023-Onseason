package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUsageId;
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

public class ManualPickupAssistCommand extends CommandBase {

    private final Swerve swerve;
    private final Superstructure superstructure;
    private final PIDController thetaController = new PIDController(5.0, 0.0, 0.0);
    private Rotation2d headingGoal;

    public ManualPickupAssistCommand(Swerve swerve, Superstructure superstructure) {
        this.swerve = swerve;
        this.superstructure = superstructure;
        addRequirements(swerve, superstructure);
    }

    @Override
    public void initialize() {
        superstructure.requestIntakeConeDoubleSubstation();
        headingGoal = DriverStation.getAlliance() == Alliance.Blue ? new Rotation2d() : new Rotation2d(Math.PI);
    }

    @Override
    public void execute() {
        Pose2d measurement = RobotContainer.poseEstimator.getLatestPose();
        double x = RobotContainer.driver.getRightY();
        double y = RobotContainer.driver.getRightX();
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
        thetaFeedback = MathUtil.clamp(thetaFeedback, -1.0, 1.0);
        swerve.requestVelocity(new ChassisSpeeds(dx, dy, thetaFeedback), true, false);
    }
    
    @Override
    public void end(boolean interrupted) { 
        swerve.requestVelocity(new ChassisSpeeds(0, 0, 0), false, false);
    }
    
}
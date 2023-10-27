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
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.Superstructure;

public class SingleSubstationDriveAssistCommand extends CommandBase {

    private final Swerve swerve;
    private final Superstructure superstructure;
    private final PIDController thetaController = new PIDController(5.0, 0.0, 0.25);
    private Rotation2d headingGoal;

    public SingleSubstationDriveAssistCommand(Swerve swerve, Superstructure superstructure) {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.swerve = swerve;
        this.superstructure = superstructure;
        addRequirements(swerve, superstructure);
    }

    @Override
    public void initialize() {
        headingGoal = DriverStation.getAlliance() == Alliance.Blue ? new Rotation2d(Math.PI / 2.0) : new Rotation2d(3 * Math.PI / 2.0);
        RobotContainer.superstructure.requestIntakeSingleSubstationCone();
    }

    @Override
    public void execute() {
        Pose2d measurement = RobotContainer.poseEstimator.getLatestPose();
        double x = BreadUtil.deadband(RobotContainer.driver.getLeftY(), 0.1);
        double y = BreadUtil.deadband(RobotContainer.driver.getLeftX(), 0.1);
        double dx;
        double dy;
      
        if (Robot.alliance == DriverStation.Alliance.Blue) {
            dx = Math.pow(-x, 1);
            dy = Math.pow(-y, 1);
        } else {
            dx = Math.pow(-x, 1) * -1;
            dy = Math.pow(-y, 1) * -1;
        }
        double thetaFeedback = thetaController.calculate(
            measurement.getRotation().getRadians(),
            headingGoal.getRadians());
        thetaFeedback = MathUtil.clamp(thetaFeedback, -5.0, 5.0);

        swerve.requestVelocity(new ChassisSpeeds(dx * 4.0, dy * 4.0, thetaFeedback), true, false);
    }
    
    @Override
    public void end(boolean interrupted) { 
    }

    
}

package frc.robot.subsystems.swerve;

import java.lang.reflect.Field;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;

public class AutoBalanceCommand extends SequentialCommandGroup {

    private final Swerve swerve;
    private final double kDeltaPitchThreshhold = 8.0;
    private final double kXGoal = 12.69; // Obtained on red alliance

    public AutoBalanceCommand(Swerve swerve) {
        addCommands(
            new InstantCommand(() -> Swerve.autoBalancing = true),
            new RunCommand(this::drive).until(this::stopDriving),
            new InstantCommand(swerve::requestXConfiguration),
            new WaitCommand(4.0)
        );
        this.swerve = swerve;
        addRequirements(swerve);
    }

    private void drive() {
        double realXGoal = kXGoal;
        double velocityAddition = 0.0;
        if (Math.abs(RobotContainer.poseEstimator.getLatestPose().getX() - realXGoal) >= 0.6) {
            if (DriverStation.getAlliance() == Alliance.Blue) {
                realXGoal = FieldConstants.fieldLength - kXGoal;
            }
            velocityAddition = MathUtil.clamp(Math.abs(RobotContainer.poseEstimator.getLatestPose().getX() - realXGoal), 0.0, 0.3);
            System.out.println(velocityAddition);
        }
        swerve.requestVelocity(new ChassisSpeeds(-0.2-velocityAddition, 0.0, 0.0), true, true);
    }

    private boolean stopDriving() {
        double realXGoal = kXGoal;
        if (DriverStation.getAlliance() == Alliance.Blue) {
            realXGoal = FieldConstants.fieldLength - kXGoal;
        }
        return swerve.getChangeInPitch() < -kDeltaPitchThreshhold && Math.abs(RobotContainer.poseEstimator.getLatestPose().getX() - realXGoal) < 0.2;
    }


    
}

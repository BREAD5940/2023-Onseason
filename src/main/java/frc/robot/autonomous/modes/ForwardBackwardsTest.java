package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

public class ForwardBackwardsTest extends SequentialCommandGroup {

    public ForwardBackwardsTest(Swerve swerve, Superstructure superstructure) {
        addRequirements(swerve, superstructure);
        addCommands(
            new TrajectoryFollowerCommand(Robot.fifteenForward, () -> Rotation2d.fromDegrees(0.0), swerve, true),
            new TrajectoryFollowerCommand(Robot.fifteenBack, swerve, false)

        );
    }
    
}

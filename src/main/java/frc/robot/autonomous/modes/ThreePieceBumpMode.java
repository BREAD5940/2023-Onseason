package frc.robot.autonomous.modes;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

import static frc.robot.Constants.Elevator.*;
import static frc.robot.Constants.Arm.*;

public class ThreePieceBumpMode extends SequentialCommandGroup {

    public ThreePieceBumpMode(Superstructure superstructure, Swerve swerve) {
        addRequirements(superstructure, swerve);
        addCommands(
            new InstantCommand(() -> swerve.requestPercent(new ChassisSpeeds(0, 0, 0), false)),
            new InstantCommand(() -> superstructure.requestPreScore(Level.MID, GamePiece.CONE)),
            new WaitUntilCommand(() -> superstructure.getElevatorHeight() > ELEVATOR_PRE_CONE_HIGH - ELEVATOR_CONE_OFFSET - 0.5),
            new InstantCommand(() -> superstructure.requestScore()),
            new WaitUntilCommand(() -> superstructure.getArmAngle() > ARM_SLAM_CONE - 10.0),
            new TrajectoryFollowerCommand(Robot.threePieceBumpA, () -> Robot.threePieceBumpA.getInitialHolonomicPose().getRotation(), swerve, true).raceWith(
                new RunCommand(() -> superstructure.requestFloorIntakeCube(() -> 0.0))
            ),
            new InstantCommand(() -> superstructure.requestFloorIntakeCube(() -> 1.0)), 
            new TrajectoryFollowerCommand(Robot.threePieceBumpB, swerve, true).alongWith(new SequentialCommandGroup(
                new WaitCommand(1.5),
                new InstantCommand(() -> superstructure.requestPreScore(Level.HIGH, GamePiece.CUBE))
            )),
            new WaitCommand(0.4),
            new InstantCommand(() -> superstructure.requestScore()), 
            new WaitCommand(0.5),
            new InstantCommand(() -> superstructure.requestFloorIntakeCube(() -> 0.0)),
            new WaitCommand(0.1),
            new TrajectoryFollowerCommand(Robot.threePieceBumpC, swerve, true),
            new InstantCommand(() -> superstructure.requestFloorIntakeCube(() -> 1.0)), 
            new TrajectoryFollowerCommand(Robot.threePieceBumpD, swerve, true).alongWith(new SequentialCommandGroup(
                new WaitCommand(1.0),
                new InstantCommand(() -> superstructure.requestPreScore(Level.MID, GamePiece.CUBE))
            )),
            new WaitCommand(0.1),
            new InstantCommand(() -> superstructure.requestScore()),
            new WaitCommand(0.5),
            new InstantCommand(() -> superstructure.requestIdle()),
            new WaitCommand(0.1),
            new TrajectoryFollowerCommand(Robot.threePieceBumpE, swerve, false),
            new RunCommand(() -> swerve.requestPercent(new ChassisSpeeds(0, 0, 0), false))
        );
    }
    
}

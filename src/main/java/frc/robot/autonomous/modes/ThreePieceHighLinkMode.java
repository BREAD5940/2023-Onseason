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
import frc.robot.subsystems.swerve.AutoPlaceCommand;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

import static frc.robot.Constants.Elevator.*;
public class ThreePieceHighLinkMode extends SequentialCommandGroup {

    public ThreePieceHighLinkMode(Superstructure superstructure, Swerve swerve) {
        addRequirements(superstructure, swerve);
        addCommands(
            new InstantCommand(() -> swerve.requestPercent(new ChassisSpeeds(0, 0, 0), false)),
            new InstantCommand(() -> superstructure.requestPreScore(Level.HIGH, GamePiece.CONE)),
            new WaitUntilCommand(() -> superstructure.atElevatorSetpoint(ELEVATOR_PRE_CONE_HIGH)),
            new InstantCommand(() -> superstructure.requestScore()),
            new WaitUntilCommand(() -> superstructure.atElevatorSetpoint(ELEVATOR_CONE_PULL_OUT_HIGH)),
            new WaitCommand(0.4),
            new TrajectoryFollowerCommand(Robot.threePieceSlowA, () -> Robot.threePieceSlowA.getInitialHolonomicPose().getRotation(), swerve, true).raceWith(
                new RunCommand(() -> superstructure.requestFloorIntakeCone())
            ),
            new InstantCommand(() -> superstructure.requestFloorIntakeCone()), 
            new TrajectoryFollowerCommand(Robot.threePieceLinkB, swerve, true),
            new AutoPlaceCommand(swerve, superstructure, () -> GamePiece.CONE, () -> Level.HIGH),
            new WaitCommand(0.5),
            new InstantCommand(() -> superstructure.requestFloorIntakeCube(() -> 0.0)),
            new WaitCommand(0.1),
            new TrajectoryFollowerCommand(Robot.threePieceSlowC, swerve, true),
            new InstantCommand(() -> superstructure.requestFloorIntakeCube(() -> 1.0)), 
            new TrajectoryFollowerCommand(Robot.threePieceSlowD, swerve, true).alongWith(new SequentialCommandGroup(
                new WaitCommand(1.0),
                new InstantCommand(() -> superstructure.requestPreScore(Level.MID, GamePiece.CUBE))
            )),
            new WaitCommand(0.1),
            new InstantCommand(() -> superstructure.requestScore()),
            new WaitCommand(0.5),
            new InstantCommand(() -> superstructure.requestIdle()),
            new WaitCommand(0.1),
            new TrajectoryFollowerCommand(Robot.threePieceSlowSetup, swerve, false),
            new RunCommand(() -> swerve.requestPercent(new ChassisSpeeds(0, 0, 0), false))
        );
    }
    
}

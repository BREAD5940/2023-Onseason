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
public class ThreePieceMode extends SequentialCommandGroup {

    public ThreePieceMode(Superstructure superstructure, Swerve swerve) {
        addRequirements(superstructure, swerve);
        addCommands(
            new InstantCommand(() -> swerve.requestPercent(new ChassisSpeeds(0, 0, 0), false)),
            new InstantCommand(() -> superstructure.requestPreScore(Level.HIGH, GamePiece.CONE)),
            new WaitUntilCommand(() -> superstructure.atElevatorSetpoint(ELEVATOR_PRE_CONE_HIGH)),
            new InstantCommand(() -> superstructure.requestScore()),
            new WaitUntilCommand(() -> superstructure.atElevatorSetpoint(ELEVATOR_CONE_PULL_OUT_HIGH)),
            new WaitCommand(0.4),
            new TrajectoryFollowerCommand(Robot.threePieceA, () -> Robot.threePieceA.getInitialHolonomicPose().getRotation(), swerve, true).raceWith(
                new RunCommand(() -> superstructure.requestFloorIntakeCube(() -> 0.0))
            ),
            new InstantCommand(() -> superstructure.requestFloorIntakeCube(() -> 1.0)), 
            new TrajectoryFollowerCommand(Robot.threePieceB, swerve, true).alongWith(new SequentialCommandGroup(
                new WaitCommand(Robot.threePieceB.getTotalTimeSeconds() - 1.05),
                new InstantCommand(() -> superstructure.requestPreScore(Level.HIGH, GamePiece.CUBE)),
                new WaitCommand(Robot.threePieceB.getTotalTimeSeconds() - 0.25),
                new InstantCommand(() -> superstructure.requestScore())
            )),
            new InstantCommand(() -> superstructure.requestFloorIntakeCube(() -> 0.0)),
            new WaitCommand(0.1),
            new TrajectoryFollowerCommand(Robot.threePieceC, swerve, true),
            new InstantCommand(() -> superstructure.requestFloorIntakeCube(() -> 1.0)), 
            new TrajectoryFollowerCommand(Robot.threePieceD, swerve, true).alongWith(new SequentialCommandGroup(
                new WaitCommand(Robot.threePieceD.getTotalTimeSeconds() - 1.88),
                new InstantCommand(() -> superstructure.requestPreScore(Level.MID, GamePiece.CUBE)),
                new WaitCommand(Robot.threePieceD.getTotalTimeSeconds() - 0.25),
                new InstantCommand(() -> superstructure.requestScore())
            )),
            new WaitCommand(0.1),
            new InstantCommand(() -> superstructure.requestIdle()),
            new WaitCommand(0.1),
            new TrajectoryFollowerCommand(Robot.threePieceSetup, swerve, false),
            new RunCommand(() -> swerve.requestPercent(new ChassisSpeeds(0, 0, 0), false))
        );
    }
    
}

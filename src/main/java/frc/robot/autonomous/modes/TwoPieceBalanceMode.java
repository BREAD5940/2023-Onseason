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
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TrajectoryFollowerCommand;

import static frc.robot.Constants.Elevator.*;

public class TwoPieceBalanceMode extends SequentialCommandGroup {

    public TwoPieceBalanceMode(Superstructure superstructure, Swerve swerve) {
        addRequirements(superstructure, swerve);
        addCommands(
            new InstantCommand(() -> swerve.requestPercent(new ChassisSpeeds(0, 0, 0), false)),
            new InstantCommand(() -> superstructure.requestPreScore(Level.HIGH, GamePiece.CONE)),
            new WaitUntilCommand(() -> superstructure.atElevatorSetpoint(ELEVATOR_PRE_CONE_HIGH)),
            new InstantCommand(() -> superstructure.requestScore()),
            new WaitUntilCommand(() -> superstructure.getSystemState() == SuperstructureState.PULL_OUT_CONE),
            new TrajectoryFollowerCommand(Robot.twoPieceBalanceA, () -> Robot.twoPieceBalanceA.getInitialHolonomicPose().getRotation(), swerve, true).raceWith(
                new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    new RunCommand(() -> superstructure.requestFloorIntakeCube(() -> 0.0))
                )
            ),
            new WaitCommand(0.05),
            new InstantCommand(() -> superstructure.requestFloorIntakeCube(() -> 1.0)), 
            new TrajectoryFollowerCommand(Robot.twoPieceBalanceB, swerve, true).alongWith(new SequentialCommandGroup(
                new WaitCommand(2.5),
                new InstantCommand(() -> superstructure.requestPreScore(Level.HIGH, GamePiece.CUBE))
            )),
            new WaitUntilCommand(() -> superstructure.atElevatorSetpoint(ELEVATOR_PRE_CUBE_HIGH)),
            new InstantCommand(() -> superstructure.requestScore()),
            new WaitCommand(0.5),
            new InstantCommand(() -> superstructure.requestIdle()), 
            new WaitCommand(0.25),
            new TrajectoryFollowerCommand(Robot.twoPieceBalanceC, swerve, false),
            // new WaitCommand(0.5),
            // new AutoBalanceCommand(swerve),
            new RunCommand(() -> swerve.requestPercent(new ChassisSpeeds(0, 0, 0), false))
        );
    }
    
}

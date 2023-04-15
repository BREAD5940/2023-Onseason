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

public class TwoPieceChargeStationMode extends SequentialCommandGroup {

    public TwoPieceChargeStationMode(Superstructure superstructure, Swerve swerve) {
        addRequirements(superstructure, swerve);
        addCommands(
                // Move forward and score the preloaded cone on the high pole
                new InstantCommand(() -> swerve.requestPercent(new ChassisSpeeds(0.0, 0, 0), false)),
                new InstantCommand(() -> superstructure.requestPreScore(Level.HIGH, GamePiece.CONE)),
                new WaitUntilCommand(() -> superstructure.atElevatorSetpoint(ELEVATOR_PRE_CONE_HIGH)),
                new InstantCommand(() -> superstructure.requestScore()),
                new WaitUntilCommand(() -> superstructure.atElevatorSetpoint(ELEVATOR_CONE_PULL_OUT_HIGH)),
                new WaitCommand(0.3),
                new InstantCommand(() -> superstructure.requestIdle()), 

                // Drive backwards over the charge station, intake the cube, and drive back over
                new TrajectoryFollowerCommand(Robot.twoPieceChargeStationA, 
                () -> Robot.twoPieceChargeStationA.getInitialHolonomicPose().getRotation(), swerve, true).raceWith(new SequentialCommandGroup(
                    new WaitCommand(2.0),
                    new RunCommand(() -> superstructure.requestFloorIntakeCube(() -> 0.0))
                )),
                new InstantCommand(() -> superstructure.requestFloorIntakeCube(() -> 1.0)),
                new WaitCommand(0.25),
                new TrajectoryFollowerCommand(Robot.twoPieceChargeStationB, swerve, true, 1.0).raceWith(new SequentialCommandGroup(
                    new WaitCommand(1.0),
                    new RunCommand(() -> superstructure.requestIdle())
                )),
                new InstantCommand(() -> superstructure.requestPreScore(Level.HIGH, GamePiece.CUBE)),
                new WaitUntilCommand(() -> superstructure.atElevatorSetpoint(ELEVATOR_PRE_CUBE_HIGH)),
                new InstantCommand(() -> superstructure.requestScore()),
                new WaitCommand(1.0),
                new InstantCommand(() -> superstructure.requestIdle()), 
                new WaitCommand(0.25),
                new TrajectoryFollowerCommand(Robot.twoPieceChargeStationC, swerve, false),
                new RunCommand(() -> swerve.requestPercent(new ChassisSpeeds(0, 0, 0), false))
        );
    }

}
package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

public class TwoPieceBalanceMode extends SequentialCommandGroup {

    public TwoPieceBalanceMode(Superstructure superstructure, Swerve swerve) {
        addRequirements(superstructure, swerve);
        addCommands(
            new InstantCommand(() -> superstructure.requestPreScore(Level.HIGH, GamePiece.CONE)),
            new WaitUntilCommand(() -> superstructure.atElevatorSetpoint(ELEVATOR_PRE_CONE_HIGH)),
            new InstantCommand(() -> superstructure.requestScore()),
            new WaitUntilCommand(() -> superstructure.atElevatorSetpoint(ELEVATOR_CONE_PULL_OUT_HIGH)),
            new InstantCommand(() -> superstructure.requestFloorIntakeCube(() -> 0.0)),
            new TrajectoryFollowerCommand(Robot.twoPieceBalanceA, () -> Robot.twoPieceBalanceA.getInitialHolonomicPose().getRotation(), swerve, true),
            new WaitCommand(0.2),
            new InstantCommand(() -> superstructure.requestFloorIntakeCube(() -> 1.0)), 
            new TrajectoryFollowerCommand(Robot.twoPieceBalanceB, swerve, true).alongWith(new SequentialCommandGroup(
                new WaitCommand(1.5),
                new InstantCommand(() -> superstructure.requestPreScore(Level.HIGH, GamePiece.CUBE))
            )),
            new WaitUntilCommand(() -> superstructure.atElevatorSetpoint(ELEVATOR_PRE_CUBE_HIGH)),
            new InstantCommand(() -> superstructure.requestScore()),
            new WaitCommand(0.35),
            new InstantCommand(() -> superstructure.requestIdle()), 
            new WaitCommand(0.25),
            new TrajectoryFollowerCommand(Robot.twoPieceBalanceC, swerve, false)
        );
    }
    
}

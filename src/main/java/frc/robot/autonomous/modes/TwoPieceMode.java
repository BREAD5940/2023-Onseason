package frc.robot.autonomous.modes;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.swerve.Swerve;

import static frc.robot.Constants.Elevator.*;

public class TwoPieceMode extends SequentialCommandGroup {

    public TwoPieceMode(Superstructure superstructure, Swerve swerve) {
        addRequirements(superstructure, swerve);
    }
    
}

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.modes.OnePieceBalanceMode;
import frc.robot.autonomous.modes.PreloadMode;
import frc.robot.autonomous.modes.ThreePieceBalanceMode;
import frc.robot.autonomous.modes.ThreePieceBumpMode;
import frc.robot.autonomous.modes.ThreePieceMode;
import frc.robot.autonomous.modes.ThrowBalanceMode;
import frc.robot.autonomous.modes.TwoPieceBalanceBumpMode;
import frc.robot.autonomous.modes.TwoPieceBalanceMode;
import frc.robot.autonomous.modes.TwoPieceBumpMode;
import frc.robot.autonomous.modes.TwoPieceMode;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.Swerve;

public class AutonomousSelector {

    private SendableChooser<SequentialCommandGroup> autonomusSelector = new SendableChooser<SequentialCommandGroup>();

    public AutonomousSelector(Swerve swerve, Superstructure superstructure) {
        autonomusSelector.setDefaultOption(
            "PRELOAD", 
            new PreloadMode(swerve, superstructure)
        );
        autonomusSelector.addOption(
            "ONE_PIECE_BALANCE", 
            new OnePieceBalanceMode(swerve, superstructure)
        );
        autonomusSelector.addOption(
            "TWO_PIECE_BALANCE", 
            new TwoPieceBalanceMode(superstructure, swerve)
        );
        autonomusSelector.addOption(
            "TWO_PIECE_BALANCE_BUMP", 
            new TwoPieceBalanceBumpMode(superstructure, swerve)
        );
        autonomusSelector.addOption(
            "TWO_PIECE", 
            new TwoPieceMode(superstructure, swerve)
        );
        autonomusSelector.addOption(
            "TWO_PIECE_BUMP", 
            new TwoPieceBumpMode(superstructure, swerve)
        );
        autonomusSelector.addOption(
            "THREE_PIECE", 
            new ThreePieceMode(superstructure, swerve)
        );
        autonomusSelector.addOption(
            "THREE_PIECE_BALANCE",
            new ThreePieceBalanceMode(superstructure, swerve)
        );
        autonomusSelector.addOption(
            "THREE_PIECE_BUMP",
            new ThreePieceBumpMode(superstructure, swerve)
        );
        autonomusSelector.addOption(
            "THROW_BALANCE",
            new ThrowBalanceMode(superstructure, swerve)
        );
        SmartDashboard.putData("Autonomus Selector", autonomusSelector);
    }

    public SequentialCommandGroup get() {
        return autonomusSelector.getSelected();
    }

}

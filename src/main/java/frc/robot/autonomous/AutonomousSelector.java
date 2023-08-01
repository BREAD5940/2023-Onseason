package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.modes.OnePieceBalanceMode;
import frc.robot.autonomous.modes.PreloadMode;
import frc.robot.autonomous.modes.ThreePieceBalanceMode;
import frc.robot.autonomous.modes.ThreePieceBumpMode;
import frc.robot.autonomous.modes.ThreePieceHighLinkMode;
import frc.robot.autonomous.modes.ThreePieceMode;
import frc.robot.autonomous.modes.ThrowBalanceMode;
import frc.robot.autonomous.modes.TwoPieceBalanceBumpMode;
import frc.robot.autonomous.modes.TwoPieceBalanceMode;
import frc.robot.autonomous.modes.TwoPieceBumpMode;
import frc.robot.autonomous.modes.TwoPieceChargeStationMode;
import frc.robot.autonomous.modes.TwoPieceMode;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.Swerve;

public class AutonomousSelector {

    private SendableChooser<SequentialCommandGroup> autonomusSelector = new SendableChooser<SequentialCommandGroup>();

    public AutonomousSelector(Swerve swerve, Superstructure superstructure) {
        autonomusSelector.setDefaultOption(
            "PRELOAD", 
            Commands.waitUntil(superstructure::homedOnce).andThen(new PreloadMode(swerve, superstructure))
        );
        autonomusSelector.addOption(
            "ONE_PIECE_BALANCE", 
            Commands.waitUntil(superstructure::homedOnce).andThen(new OnePieceBalanceMode(swerve, superstructure))
        );
        autonomusSelector.addOption(
            "TWO_PIECE_BALANCE", 
            Commands.waitUntil(superstructure::homedOnce).andThen(new TwoPieceBalanceMode(superstructure, swerve))
        );
        autonomusSelector.addOption(
            "TWO_PIECE_BALANCE_BUMP", 
            Commands.waitUntil(superstructure::homedOnce).andThen(new TwoPieceBalanceBumpMode(superstructure, swerve))
        );
        autonomusSelector.addOption(
            "TWO_PIECE", 
            Commands.waitUntil(superstructure::homedOnce).andThen(new TwoPieceMode(superstructure, swerve))
        );
        autonomusSelector.addOption(
            "TWO_PIECE_BUMP", 
            Commands.waitUntil(superstructure::homedOnce).andThen(new TwoPieceBumpMode(superstructure, swerve))
        );
        autonomusSelector.addOption(
            "THREE_PIECE", 
            Commands.waitUntil(superstructure::homedOnce).andThen(new ThreePieceMode(superstructure, swerve))
        );
        autonomusSelector.addOption(
            "THREE_PIECE_BALANCE",
            Commands.waitUntil(superstructure::homedOnce).andThen(new ThreePieceBalanceMode(superstructure, swerve))
        );
        autonomusSelector.addOption(
            "THREE_PIECE_BUMP",
            Commands.waitUntil(superstructure::homedOnce).andThen(new ThreePieceBumpMode(superstructure, swerve))
        );
        autonomusSelector.addOption(
            "THROW_BALANCE",
            Commands.waitUntil(superstructure::homedOnce).andThen(new ThrowBalanceMode(superstructure, swerve))
        );
        autonomusSelector.addOption(
            "TWO_PIECE_CHARGE_STATION",
            Commands.waitUntil(superstructure::homedOnce).andThen(new TwoPieceChargeStationMode(superstructure, swerve))
        );
        autonomusSelector.addOption(
            "THREE_PIECE_HIGH_LINK",
            Commands.waitUntil(superstructure::homedOnce).andThen(new ThreePieceHighLinkMode(superstructure, swerve))
        );
        SmartDashboard.putData("Autonomus Selector", autonomusSelector);
    }

    public SequentialCommandGroup get() {
        return autonomusSelector.getSelected();
    }

}

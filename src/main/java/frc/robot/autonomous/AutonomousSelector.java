package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.modes.ThreePieceMode;
import frc.robot.autonomous.modes.TwoPieceBalanceBumpMode;
import frc.robot.autonomous.modes.TwoPieceBalanceMode;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.Swerve;

public class AutonomousSelector {

    private SendableChooser<SequentialCommandGroup> autonomusSelector = new SendableChooser<SequentialCommandGroup>();

    public AutonomousSelector(Swerve swerve, Superstructure superstructure) {
        autonomusSelector.setDefaultOption(
            "DO_NOTHING", 
            new SequentialCommandGroup()
        );
        autonomusSelector.addOption(
            "TWO_PIECE_BALANCE", 
            new TwoPieceBalanceMode(superstructure, swerve)
        );
        autonomusSelector.addOption(
            "THREE_PIECE", 
            new ThreePieceMode(superstructure, swerve)
        );
        autonomusSelector.addOption(
            "THREE_PIECE_BUMP", 
            new TwoPieceBalanceBumpMode(superstructure, swerve)
        );
        SmartDashboard.putData("Autonomus Selector", autonomusSelector);
    }

    public SequentialCommandGroup get() {
        return autonomusSelector.getSelected();
    }

}

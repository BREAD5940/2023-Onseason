package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutoBalanceCommand extends SequentialCommandGroup {

    private final Swerve swerve;
    private final double kDeltaPitchThreshhold = 8.0;

    public AutoBalanceCommand(Swerve swerve) {
        addCommands(
            new InstantCommand(() -> Swerve.autoBalancing = true),
            new InstantCommand(() -> swerve.requestVelocity(new ChassisSpeeds(-0.2, 0.0, 0.0), true, true)), 
            new WaitUntilCommand(this::stopDriving),
            new InstantCommand(swerve::requestXConfiguration),
            new WaitCommand(4.0)
        );
        this.swerve = swerve;
        addRequirements(swerve);
    }

    private boolean stopDriving() {
        return swerve.getChangeInPitch() < -kDeltaPitchThreshhold;
    }


    
}

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.Level;

public class OperatorControls {

  private Level lastSelectedLevel = Level.HIGH;
  private GamePiece lastSelectedGamePiece = GamePiece.CONE;
  private GenericHID controller;

  public OperatorControls(GenericHID controller) {
    this.controller = controller;
  }

  public void updateSelection() {
    if (controller.getRawButton(1)) {
      lastSelectedLevel = Level.HIGH;
    } else if (controller.getRawButton(2)) {
      lastSelectedLevel = Level.MID;
    } else if (controller.getRawButton(3)) {
      lastSelectedLevel = Level.LOW;
    }

    if (controller.getRawButton(4)) {
      lastSelectedGamePiece = GamePiece.CONE;
    } else if (controller.getRawButton(5)) {
      lastSelectedGamePiece = GamePiece.CUBE;
    }

    Logger.getInstance().recordOutput("OperatorControls/lastSelectedGamePiece", lastSelectedGamePiece.toString());
    Logger.getInstance().recordOutput("OperatorControls/lastSelectedLevel", lastSelectedLevel.name());

  }

  public GamePiece getLastSelectedGamePiece() {
    return lastSelectedGamePiece;
  }

  public Level getLastSelectedLevel() {
    return lastSelectedLevel;
  }
}

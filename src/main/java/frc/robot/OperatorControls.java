package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.Superstructure.Level;

public class OperatorControls {

  private Level lastSelectedLevel = Level.HIGH;
  private int lastSelectedScoringLocation = 1;
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
      lastSelectedScoringLocation = 1;
    } else if (controller.getRawButton(5)) {
      lastSelectedScoringLocation = 2;
    } else if (controller.getRawButton(6)) {
      lastSelectedScoringLocation = 3;
    } else if (controller.getRawButton(7)) {
      lastSelectedScoringLocation = 4;
    } else if (controller.getRawButton(8)) {
      lastSelectedScoringLocation = 5;
    } else if (controller.getRawButton(9)) {
      lastSelectedScoringLocation = 6;
    } else if (controller.getRawButtonPressed(10)) {
      lastSelectedScoringLocation = 7;
    } else if (controller.getRawButtonPressed(11)) {
      lastSelectedScoringLocation = 8;
    } else if (controller.getRawButtonPressed(12)) {
      lastSelectedScoringLocation = 9;
    }

    Logger.getInstance().recordOutput("OperatorControls/lastSelectedScoringLocation", lastSelectedScoringLocation);
    Logger.getInstance().recordOutput("OperatorControls/lastSelectedLevel", lastSelectedLevel.name());

  }

  public int getLastSelectedScoringLocation() {
    return lastSelectedScoringLocation;
  }

  public Level getLastSelectedLevel() {
    return lastSelectedLevel;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {

  public final XboxController driver = new XboxController(0);
  public final Swerve swerve = new Swerve();

  public RobotContainer() { }

  public Command getAutonomousCommand() {
    return null;
  }
}

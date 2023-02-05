// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.elevatorarm.ArmIO;
import frc.robot.subsystems.elevatorarm.ArmIOTalonFX;
import frc.robot.subsystems.elevatorarm.ElevatorIO;
import frc.robot.subsystems.elevatorarm.ElevatorIOTalonFX;
import frc.robot.subsystems.endeffector.EndEffectorIO;
import frc.robot.subsystems.endeffector.EndEffectorIOSparkMax;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {

  public static final XboxController driver = new XboxController(0);
  public static final XboxController operator = new XboxController(1);
  public static final Swerve swerve = new Swerve();
  public static final ElevatorIO elevatorIO = new ElevatorIOTalonFX();
  public static final ArmIO armIO = new ArmIOTalonFX();
  public static final EndEffectorIO endEffectorIO = new EndEffectorIOSparkMax();
  public static final Superstructure superstructure = new Superstructure(elevatorIO, armIO, endEffectorIO);

  public RobotContainer() { }

  public Command getAutonomousCommand() {
    return null;
  }
}

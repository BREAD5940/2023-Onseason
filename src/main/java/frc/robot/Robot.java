// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.Level;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  PowerDistribution pdp;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    Logger.getInstance().recordMetadata("ProjectName", "2023-Alpha"); // Set a metadata value

    if (isReal()) {
        Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
        Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        pdp = new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }
    
    Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    RobotContainer.superstructure.zeroSensors();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    RobotContainer.superstructure.requestHome();
  }

  @Override
  public void teleopPeriodic() {
    double x = m_robotContainer.driver.getRightY();
    double y = m_robotContainer.driver.getRightX();
    double omega = m_robotContainer.driver.getLeftX();

    // Movement Outputs
    double dx = Math.abs(x) > 0.075 ? Math.pow(-x, 1) : 0.0;
    double dy = Math.abs(y) > 0.075 ? Math.pow(-y, 1) : 0.0;
    double rot = Math.abs(omega) > 0.1 ? Math.pow(-omega, 3) * 0.5 : 0.0;
    m_robotContainer.swerve.requestPercent(new ChassisSpeeds(dx, dy, rot), true);

    if (m_robotContainer.driver.getAButtonPressed()) {
      m_robotContainer.swerve.reset(new Pose2d());
    }

    // Sets the 0 of the robot
    if (m_robotContainer.driver.getAButtonPressed()) {
      m_robotContainer.swerve.reset(new Pose2d());
    }

    if (RobotContainer.operator.getRightBumperPressed()) {
      RobotContainer.superstructure.requestIntakeConeDoubleSubstation();
    }

    if (RobotContainer.operator.getLeftBumperPressed()) {
      RobotContainer.superstructure.requestIntakeCubeDoubleSubstation();
    }

    if (RobotContainer.operator.getAButtonPressed()) {
      RobotContainer.superstructure.requestPreScore(Level.HIGH, GamePiece.CONE);
    }

    if (RobotContainer.operator.getBButtonPressed()) {
      RobotContainer.superstructure.requestPreScore(Level.MID, GamePiece.CONE);
    }
    
    if (RobotContainer.operator.getXButtonPressed()) {
      RobotContainer.superstructure.requestPreScore(Level.HIGH, GamePiece.CUBE);
    }

    if (RobotContainer.operator.getYButtonPressed()) {
      RobotContainer.superstructure.requestPreScore(Level.MID, GamePiece.CUBE);
    }

    if (RobotContainer.operator.getRightStickButtonPressed()) {
      RobotContainer.superstructure.requestScore();
    }

    if (RobotContainer.operator.getLeftStickButtonPressed()) {
      RobotContainer.superstructure.requestIdle();
    }


  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}

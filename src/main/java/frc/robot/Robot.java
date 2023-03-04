// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.Level;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private boolean intakeDeployed = false;
  private boolean spitDeployed = false;

  PowerDistribution pdp;

  private boolean homed = false;
  private boolean intakeTriggered = false;
  private boolean coneIntakeTriggered = false;

  public static PathPlannerTrajectory threePieceA;
  public static PathPlannerTrajectory threePieceB;
  public static PathPlannerTrajectory threePieceC;
  public static PathPlannerTrajectory threePieceD;
  public static PathPlannerTrajectory threePieceE;

  public static PathPlannerTrajectory twoPieceBalanceA;
  public static PathPlannerTrajectory twoPieceBalanceB;
  public static PathPlannerTrajectory twoPieceBalanceC;

  public static PathPlannerTrajectory twoPieceBalanceBumpA;
  public static PathPlannerTrajectory twoPieceBalanceBumpB;
  public static PathPlannerTrajectory twoPieceBalanceBumpC;

  public static PathPlannerTrajectory test;

  public static int scoringSquare = 0; // Number 0-2
  public static int scoringSpot = 0; // Number 0-2
  public static Level scoringLevel = Level.HIGH;

  public static Alliance alliance = DriverStation.Alliance.Red;

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
      Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save
                                                                                                          // outputs to
                                                                                                          // a new log
    }

    Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may
                                  // be added.

    RobotContainer.superstructure.zeroSensors();

    threePieceA = PathPlanner.loadPath("Three Piece A", new PathConstraints(4.0, 2.0));
    threePieceB = PathPlanner.loadPath("Three Piece B", new PathConstraints(3.0, 2.0));
    threePieceC = PathPlanner.loadPath("Three Piece C", new PathConstraints(4.0, 2.5));
    threePieceD = PathPlanner.loadPath("Three Piece D", new PathConstraints(4.0, 2.0));
    threePieceE = PathPlanner.loadPath("Three Piece E", new PathConstraints(4.0, 2.0));
    // threePieceA = PathPlanner.loadPath("Three Piece A", new PathConstraints(1.0, 0.5));
    // threePieceB = PathPlanner.loadPath("Three Piece B", new PathConstraints(1.0, 0.5));
    // threePieceC = PathPlanner.loadPath("Three Piece C", new PathConstraints(1.0, 0.5));
    // threePieceD = PathPlanner.loadPath("Three Piece D", new PathConstraints(1.0, 0.5));
    // threePieceE = PathPlanner.loadPath("Three Piece E", new PathConstraints(1.0, 0.5));
    twoPieceBalanceA = PathPlanner.loadPath("Two Piece Balance A", new PathConstraints(4.5, 2.5));
    twoPieceBalanceB = PathPlanner.loadPath("Two Piece Balance B", new PathConstraints(4.5, 2.5));
    twoPieceBalanceC = PathPlanner.loadPath("Two Piece Balance C", new PathConstraints(2.0, 2.0));
    twoPieceBalanceBumpA = PathPlanner.loadPath("Two Piece Balance Bump A", new PathConstraints(2.0, 1.0));
    twoPieceBalanceBumpB = PathPlanner.loadPath("Two Piece Balance Bump B", new PathConstraints(2.0, 1.0));
    twoPieceBalanceBumpC = PathPlanner.loadPath("Two Piece Balance Bump C", new PathConstraints(2.0, 2.0));
    test = PathPlanner.loadPath("Test", new PathConstraints(1.0, 0.5));
    RobotContainer.swerve.resetAllToAbsolute();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Logger.getInstance().recordOutput("Alliance Color", alliance.toString());
    RobotContainer.operatorControls.updateSelection();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (!homed) {
      RobotContainer.superstructure.requestHome();
      homed = true;
    }

    alliance = DriverStation.getAlliance();
  }

  @Override
  public void teleopPeriodic() {
    if (RobotContainer.driver.getLeftTriggerAxis() > 0.1 && !coneIntakeTriggered) {
      RobotContainer.superstructure.requestFloorIntakeCone();
      coneIntakeTriggered = true;
    } else {
      coneIntakeTriggered = false;
    }

    if (RobotContainer.driver.getRightTriggerAxis() > 0.05 && !intakeTriggered) {
      Supplier<Double> pressure = () -> {
        if (RobotContainer.driver.getRightStickButton() && RobotContainer.driver.getRightTriggerAxis() > 0.95 ) {
          return (RobotContainer.driver.getRightTriggerAxis() - 0.95) * (1.0 / 0.05);
        } else {
          return 0.0;
        }
      };
      RobotContainer.superstructure.requestFloorIntakeCube(pressure);
      intakeTriggered = true;
    }

    if (RobotContainer.driver.getRightTriggerAxis() <= 0.05 && intakeTriggered) {
      intakeTriggered = false;
      RobotContainer.superstructure.requestIdle();
    }

    if (RobotContainer.driver.getRightBumperPressed()) {
      RobotContainer.superstructure.requestIntakeConeDoubleSubstation();
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

    // if (RobotContainer.driver.getRightStickButtonPressed()) {
    //   RobotContainer.superstructure.requestIdle();
    //   coneIntakeTriggered = false;
    // }

    if (RobotContainer.operator.getRightTriggerAxis() > 0.1) {
      RobotContainer.climberIO.setPercent(1.0);
    } else if (RobotContainer.operator.getLeftTriggerAxis() > 0.1) {
      RobotContainer.climberIO.setPercent(-1.0);
    } else {
      RobotContainer.climberIO.setPercent(0.0);
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}

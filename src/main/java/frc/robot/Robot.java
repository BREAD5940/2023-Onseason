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
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.FaultChecker.FaultCheckerTreaded;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private boolean intakeDeployed = false;
  private boolean spitDeployed = false;

  PowerDistribution pdp;

  private boolean homed = false;
  private boolean intakeTriggered = false;

  public static PathPlannerTrajectory threePieceA;
  public static PathPlannerTrajectory threePieceB;
  public static PathPlannerTrajectory threePieceC;
  public static PathPlannerTrajectory threePieceD;
  public static PathPlannerTrajectory threePieceBalance;
  public static PathPlannerTrajectory threePieceSetup;

  public static PathPlannerTrajectory threePieceBumpA;
  public static PathPlannerTrajectory threePieceBumpB;
  public static PathPlannerTrajectory threePieceBumpC;
  public static PathPlannerTrajectory threePieceBumpD;
  public static PathPlannerTrajectory threePieceBumpE;

  public static PathPlannerTrajectory twoPieceBalanceA;
  public static PathPlannerTrajectory twoPieceBalanceB;
  public static PathPlannerTrajectory twoPieceBalanceC;

  public static PathPlannerTrajectory twoPieceBalanceBumpA;
  public static PathPlannerTrajectory twoPieceBalanceBumpB;
  public static PathPlannerTrajectory twoPieceBalanceBumpC;

  public static PathPlannerTrajectory onePieceBalanceA;
  public static PathPlannerTrajectory onePieceBalanceB;

  public static PathPlannerTrajectory test;

  FaultCheckerTreaded faultChecker;

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
        Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }
    
    Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

	faultChecker = new FaultCheckerTreaded();
	faultChecker.start();

    RobotContainer.superstructure.zeroSensors();

	threePieceA = PathPlanner.loadPath("Three Piece A", new PathConstraints(4.0, 3.0));
    threePieceB = PathPlanner.loadPath("Three Piece B", new PathConstraints(4.0, 3.0));
    threePieceC = PathPlanner.loadPath("Three Piece C", new PathConstraints(4.0, 3.0));
    threePieceD = PathPlanner.loadPath("Three Piece D", new PathConstraints(4.0, 3.0));
    threePieceBalance = PathPlanner.loadPath("Three Piece Balance", new PathConstraints(4.0, 3.0));
    threePieceSetup = PathPlanner.loadPath("Three Piece Setup", new PathConstraints(4.0, 3.0));

    threePieceBumpA = PathPlanner.loadPath("Three Piece Bump A", new PathConstraints(4.0, 3.0));
    threePieceBumpB = PathPlanner.loadPath("Three Piece Bump B", new PathConstraints(4.0, 3.0));
    threePieceBumpC = PathPlanner.loadPath("Three Piece Bump C", new PathConstraints(4.0, 3.0));
    threePieceBumpD = PathPlanner.loadPath("Three Piece Bump D", new PathConstraints(4.0, 3.0));
    threePieceBumpE = PathPlanner.loadPath("Three Piece Bump E", new PathConstraints(4.0, 3.0));

    twoPieceBalanceA = PathPlanner.loadPath("Two Piece Balance A", new PathConstraints(2.0, 2.0));
    twoPieceBalanceB = PathPlanner.loadPath("Two Piece Balance B", new PathConstraints(2.0, 2.0));
    twoPieceBalanceC = PathPlanner.loadPath("Two Piece Balance C", new PathConstraints(2.0, 2.0));

    twoPieceBalanceBumpA = PathPlanner.loadPath("Two Piece Balance Bump A", new PathConstraints(2.0, 2.0));
    twoPieceBalanceBumpB = PathPlanner.loadPath("Two Piece Balance Bump B", new PathConstraints(2.0, 2.0));
    twoPieceBalanceBumpC = PathPlanner.loadPath("Two Piece Balance Bump C", new PathConstraints(2.0, 2.0));

    onePieceBalanceA = PathPlanner.loadPath("One Piece Balance A", new PathConstraints(2.0, 2.0));
    onePieceBalanceB = PathPlanner.loadPath("One Piece Balance B", new PathConstraints(2.0, 2.0));
    RobotContainer.swerve.resetAllToAbsolute();
    m_robotContainer.configureAutonomousSelector(); // Needed down here so auto paths e
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

    if (!homed) {
      RobotContainer.superstructure.requestHome();
      homed = true;
    }
  }

  @Override
  public void teleopPeriodic() {
    if (RobotContainer.operator.getRightBumperPressed()) {
      RobotContainer.superstructure.requestFloorIntakeCone();
    }

    if (RobotContainer.operator.getRightTriggerAxis() > 0.05 && !intakeTriggered) {
      Supplier<Double> pressure = () -> {
        if (RobotContainer.operator.getRightTriggerAxis() < 0.75) {
           return 0.0;
        } else {
          return (RobotContainer.operator.getRightTriggerAxis() - 0.75) * (1.0/0.25);
        }
      };
      RobotContainer.superstructure.requestFloorIntakeCube(pressure);
      intakeTriggered = true;
    } 

    if (RobotContainer.operator.getRightTriggerAxis() <= 0.05 && intakeTriggered) {
      intakeTriggered = false;
      RobotContainer.superstructure.requestIdle();
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
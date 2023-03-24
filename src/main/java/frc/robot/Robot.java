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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.FaultChecker.FaultChecker;
import frc.robot.FaultChecker.FaultCheckerTreaded;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.climber.Climber.ClimberStates;
import frc.robot.subsystems.vision.northstar.AprilTagVision;
import frc.robot.subsystems.vision.visionTest.*;
import edu.wpi.first.math.Pair;
import frc.robot.subsystems.swerve.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds.*;

public class Robot extends LoggedRobot {
	private Command m_autonomousCommand;

	private RobotContainer m_robotContainer;
	private boolean intakeDeployed = false;
	private boolean spitDeployed = false;

	PowerDistribution pdp;

	private boolean requestedHome = false;
	private boolean intakeTriggered = false;
	private boolean coneIntakeTriggered = false;

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

	public static Alliance alliance = DriverStation.Alliance.Red;

	int loopcounter = 0;

	FaultCheckerTreaded ethernetLogger;

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
			String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the
															// user)
			Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
			Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
			// Save outputs to a new log
		}
		ethernetLogger = new FaultCheckerTreaded();
		ethernetLogger.start();

		Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may
										// be added.

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
		m_robotContainer.configureAutonomousSelector(); // Needed down here so auto paths exist when the selector is
														// created
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

		if (!requestedHome) {
			RobotContainer.superstructure.requestHome();
			requestedHome = true;
		}

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}

		alliance = DriverStation.getAlliance();
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		if (!requestedHome) {
			RobotContainer.superstructure.requestHome();
			requestedHome = true;
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
				if (RobotContainer.driver.getYButton() && RobotContainer.driver.getRightTriggerAxis() > 0.95) {
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

		if (RobotContainer.operator.getLeftStickButtonPressed()) {
			RobotContainer.superstructure.requestIdle();
		}

		if (RobotContainer.operator.getPOV() == 0 || RobotContainer.operator.getPOV() == 45
				|| RobotContainer.operator.getPOV() == 315) {
			RobotContainer.superstructure.requestSpit();
		}

		if (RobotContainer.operator.getRawButtonPressed(XboxController.Button.kStart.value)) {
			RobotContainer.superstructure.requestPreScore(Level.LOW, GamePiece.CUBE);
		}

		if (RobotContainer.operator.getRightBumper() && RobotContainer.operator.getLeftBumper()) {
			RobotContainer.climber.requestDeploy();
		}

		if (RobotContainer.climber.getSystemState() != ClimberStates.RETRACTED
				&& RobotContainer.climber.getSystemState() != ClimberStates.DEPLOYING) {
			if (RobotContainer.operator.getRightTriggerAxis() > 0.1) {
				RobotContainer.climber.requestRun(RobotContainer.operator.getRightTriggerAxis());
			} else if (RobotContainer.operator.getLeftTriggerAxis() > 0.1) {
				RobotContainer.climber.requestRun(-RobotContainer.operator.getLeftTriggerAxis());
			} else {
				RobotContainer.climber.requestRun(0.0);
			}
		}

		AprilTagVision.setTrustLevel(RobotContainer.operator.getPOV() == 135 || RobotContainer.operator.getPOV() == 180
				|| RobotContainer.operator.getPOV() == 225);
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
		RobotContainer.faultChecker.update();
		boolean driveFlipper = false;
		RobotContainer.northstarVision.periodic();
		loopcounter++;
		RobotContainer.superstructure.resetError();
		if (loopcounter % 50 == 0) {
			if (driveFlipper) {
				RobotContainer.swerve.requestPercent(new ChassisSpeeds(0.1, 0, 0), false);
				driveFlipper = false;
			} else {
				RobotContainer.swerve.requestPercent(new ChassisSpeeds(0, 0.1, 0), false);
				driveFlipper = true;
			}
		}
	}

	@Override
	public void simulationInit() {
	}

	@Override
	public void simulationPeriodic() {
	}

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commons.BreadUtil;
import frc.robot.commons.PoseEstimator;
import frc.robot.drivers.LEDs;
import frc.robot.autonomous.AutonomousSelector;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.elevatorarm.ArmIO;
import frc.robot.subsystems.elevatorarm.ArmIOTalonFX;
import frc.robot.subsystems.elevatorarm.ElevatorIO;
import frc.robot.subsystems.elevatorarm.ElevatorIOTalonFX;
import frc.robot.subsystems.endeffector.EndEffectorIO;
import frc.robot.subsystems.endeffector.EndEffectorIOTalonFX;
import frc.robot.subsystems.floorintake.FloorIntakeIO;
import frc.robot.subsystems.floorintake.FloorIntakeIOTalonFX;
import frc.robot.subsystems.swerve.AutoPickupRoutine;
import frc.robot.subsystems.swerve.AutoPlaceCommand;
import frc.robot.subsystems.swerve.ManualPickupAssistCommand;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.limelight.LimelightDetectionsClassifier;
import frc.robot.subsystems.vision.northstar.AprilTagVision;
import frc.robot.subsystems.vision.northstar.AprilTagVisionIO;
import frc.robot.subsystems.vision.northstar.AprilTagVisionIONorthstar;
import frc.robot.subsystems.vision.visionTest.CameraPoseTester;
import frc.robot.subsystems.faultChecker.FaultChecker;
import frc.robot.subsystems.faultChecker.Testmode;


public class RobotContainer {

  public static final XboxController driver = new XboxController(0);
  public static final XboxController operator = new XboxController(1);
  public static final GenericHID keyboard = new GenericHID(2);
  public static final OperatorControls operatorControls = new OperatorControls(keyboard);

  public static final Swerve swerve = new Swerve();
  public static final ElevatorIO elevatorIO = new ElevatorIOTalonFX();
  public static final ArmIO armIO = new ArmIOTalonFX();
  public static final EndEffectorIO endEffectorIO = new EndEffectorIOTalonFX();
  public static final FloorIntakeIO floorIntakeIO = new FloorIntakeIOTalonFX();
  public static final Superstructure superstructure = new Superstructure(elevatorIO, armIO, endEffectorIO,
      floorIntakeIO);
  public static final LimelightDetectionsClassifier limelightVision = new LimelightDetectionsClassifier("limelight");
  public static final AprilTagVisionIO leftCamera = new AprilTagVisionIONorthstar("northstar-left");
  public static final AprilTagVisionIO rightCamera = new AprilTagVisionIONorthstar("northstar-right");
  public static final AprilTagVisionIO centerCamera = new AprilTagVisionIONorthstar("northstar-center");
  public static final AprilTagVision northstarVision = new AprilTagVision(leftCamera, rightCamera, centerCamera);
  public static final PoseEstimator poseEstimator = new PoseEstimator(VecBuilder.fill(0.005, 0.005, 0.0005));
  
  public static final ClimberIOTalonFX climberIO = new ClimberIOTalonFX();
  public static final Climber climber = new Climber(climberIO);
  public static final LEDs leds = new LEDs(0, 74);

  public static final Testmode testmode = new Testmode();
  public static final FaultChecker faultChecker = new FaultChecker();
  public static final CameraPoseTester cameraPoseTester = new CameraPoseTester(northstarVision);
  

  private static AutonomousSelector autonomousSelector;

  public RobotContainer() {
    configureControls();
    configureNorthstarVision();
	faultChecker.start();
  }

  private void configureControls() {
    swerve.setDefaultCommand(new RunCommand(() -> {
      double x = BreadUtil.deadband(driver.getRightY(), 0.1);
      double y = BreadUtil.deadband(driver.getRightX(), 0.1);
      double omega = BreadUtil.deadband(driver.getLeftX(), 0.1);

      // Movement Outputs
      double scale = RobotContainer.driver.getLeftBumper() ? 0.25 : 1.0;
      double dx;
      double dy;
      if (Robot.alliance == DriverStation.Alliance.Blue) {
        dx = Math.pow(-x, 1) * scale;
        dy = Math.pow(-y, 1) * scale;
        
      } else {
        dx = Math.pow(-x, 1) * scale * -1;
        dy = Math.pow(-y, 1) * scale * -1;
      }
      double rot = Math.pow(-omega, 3) * 1.5 * scale;
      swerve.requestPercent(new ChassisSpeeds(dx, dy, rot), true);

      // Sets the 0 of the robot
      if (driver.getRawButtonPressed(XboxController.Button.kStart.value)) {
        if (DriverStation.getAlliance() == Alliance.Blue) {
          poseEstimator.resetPose(new Pose2d());
        } else {
          poseEstimator.resetPose(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));
        }
      }
    }, swerve));

    new JoystickButton(operator, XboxController.Button.kBack.value).onTrue(new InstantCommand(() -> superstructure.requestHome()));

    // superstructure.setDefaultCommand(new RunCommand(() -> {
    // if (RobotContainer.operator.getRightBumperPressed()) {
    // RobotContainer.superstructure.requestFloorIntakeCone();
    // }

    // if (RobotContainer.operator.getLeftBumperPressed()) {
    // RobotContainer.superstructure.requestFloorIntakeCube();
    // }

    // if (RobotContainer.operator.getAButtonPressed()) {
    // RobotContainer.superstructure.requestPreScore(Level.HIGH, GamePiece.CONE);
    // }

    // if (RobotContainer.operator.getBButtonPressed()) {
    // RobotContainer.superstructure.requestPreScore(Level.MID, GamePiece.CONE);
    // }

    // if (RobotContainer.operator.getXButtonPressed()) {
    // RobotContainer.superstructure.requestPreScore(Level.HIGH, GamePiece.CUBE);
    // }

    // if (RobotContainer.operator.getYButtonPressed()) {
    // RobotContainer.superstructure.requestPreScore(Level.MID, GamePiece.CUBE);
    // }

    // if (RobotContainer.operator.getRightStickButtonPressed()) {
    // RobotContainer.superstructure.requestScore();
    // }

    // if (RobotContainer.operator.getLeftStickButtonPressed()) {
    // RobotContainer.superstructure.requestIdle();
    // }
    // }, superstructure));

    // new JoystickButton(operator,
    // XboxController.Button.kRightBumper.value).onTrue(
    // new InstantCommand(() -> superstructure.requestFloorIntakeCone())
    // );

    new JoystickButton(driver, XboxController.Button.kX.value)
        .whileTrue(new AutoPlaceCommand(swerve, superstructure, () -> operatorControls.getLastSelectedScoringLocation(), () -> operatorControls.getLastSelectedLevel()));

    // new JoystickButton(driver, XboxController.Button.kA.value)
    //     .whileTrue(new AutoPickupRoutine(driver::getAButton, driver::getBButton, swerve, superstructure));

    // new JoystickButton(driver, XboxController.Button.kB.value)
    //     .whileTrue(new AutoPickupRoutine(driver::getAButton, driver::getBButton, swerve, superstructure));

    new JoystickButton(driver, XboxController.Button.kA.value)
    .whileTrue(new ManualPickupAssistCommand(swerve, superstructure));

  }

  private void configureNorthstarVision() {
    northstarVision.setDataInterfaces(poseEstimator::getLatestPose, poseEstimator::addVisionData);
  }

  public Command getAutonomousCommand() {
    return autonomousSelector.get();
  }

  public void configureAutonomousSelector() {
    autonomousSelector = new AutonomousSelector(swerve, superstructure);
  }

}

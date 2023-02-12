// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commons.PoseEstimator;
import frc.robot.autonomous.modes.ThreePieceMode;
import frc.robot.autonomous.modes.TwoPieceBalanceMode;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.GamePiece;
import frc.robot.subsystems.Superstructure.Level;
import frc.robot.subsystems.elevatorarm.ArmIO;
import frc.robot.subsystems.elevatorarm.ArmIOTalonFX;
import frc.robot.subsystems.elevatorarm.ElevatorIO;
import frc.robot.subsystems.elevatorarm.ElevatorIOTalonFX;
import frc.robot.subsystems.endeffector.EndEffectorIO;
import frc.robot.subsystems.endeffector.EndEffectorIOSparkMax;
import frc.robot.subsystems.floorintake.FloorIntakeIO;
import frc.robot.subsystems.floorintake.FloorIntakeIOTalonFX;
import frc.robot.subsystems.swerve.AutoPickupRoutine;
import frc.robot.subsystems.swerve.AutoPlaceCommand;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.northstar.AprilTagVision;
import frc.robot.subsystems.vision.northstar.AprilTagVisionIO;
import frc.robot.subsystems.vision.northstar.AprilTagVisionIONorthstar;

import static frc.robot.FieldConstants.*;

public class RobotContainer {

  public static final XboxController driver = new XboxController(0);
  public static final XboxController operator = new XboxController(1);
  public static final GenericHID keyboard = new GenericHID(2);

  public static final Swerve swerve = new Swerve();
  public static final ElevatorIO elevatorIO = new ElevatorIOTalonFX();
  public static final ArmIO armIO = new ArmIOTalonFX();
  public static final EndEffectorIO endEffectorIO = new EndEffectorIOSparkMax();
  public static final FloorIntakeIO floorIntakeIO = new FloorIntakeIOTalonFX();
<<<<<<< HEAD
  public static final Superstructure superstructure = new Superstructure(elevatorIO, armIO, endEffectorIO,
      floorIntakeIO);
  private static final AprilTagVisionIO leftCamera = new AprilTagVisionIONorthstar("northstar-left");
  private static final AprilTagVisionIO rightCamera = new AprilTagVisionIONorthstar("northstar-right");
  public static final AprilTagVision northstarVision = new AprilTagVision(leftCamera, rightCamera);
  public static final PoseEstimator poseEstimator = new PoseEstimator(VecBuilder.fill(0.005, 0.005, 0.0005));
=======
  public static final Superstructure superstructure = new Superstructure(elevatorIO, armIO, endEffectorIO, floorIntakeIO);
  public static final OperatorControls operatorControls = new OperatorControls(keyboard);
>>>>>>> 6e2a21a (add keyboard controls for operator)

  public RobotContainer() {
    configureControls();
    configureNorthstarVision();
  }

  private void configureControls() {
    swerve.setDefaultCommand(new RunCommand(() -> {
      double x = driver.getRightY();
      double y = driver.getRightX();
      double omega = driver.getLeftX();

      // Movement Outputs
      double scale = RobotContainer.driver.getLeftBumper() ? 0.25 : 1.0;
      double dx = Math.abs(x) > 0.075 ? Math.pow(-x, 1) * scale : 0.0;
      double dy = Math.abs(y) > 0.075 ? Math.pow(-y, 1) * scale : 0.0;
      double rot = Math.abs(omega) > 0.1 ? Math.pow(-omega, 3) * 0.75 * scale : 0.0;
      swerve.requestPercent(new ChassisSpeeds(dx, dy, rot), true);

      // Sets the 0 of the robot
      if (driver.getAButtonPressed()) {
        poseEstimator.resetPose(new Pose2d());
      }
    }, swerve));

<<<<<<< HEAD
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
=======
    superstructure.setDefaultCommand(new RunCommand(() -> {
      operatorControls.updateSelection();
      GamePiece selectedGamePiece = GamePiece.CONE;
  
      if (FieldConstants.canScoreCube(operatorControls.getLastSelectedLevel(), operatorControls.getLastSelectedColumn())) {
        selectedGamePiece = GamePiece.CUBE;
      }
      // if (driver.getRightBumperPressed()) {
      //   RobotContainer.superstructure.requestPreScore(operatorControls.getLastSelectedLevel(), selectedGamePiece);
      // }
  
      if (driver.getRightTriggerAxis() > 0.5 ) {
        RobotContainer.superstructure.requestScore();
      }
  
      if (driver.getLeftBumperPressed()) {
        RobotContainer.superstructure.requestIntakeConeDoubleSubstation();
      }
      
      if (driver.getLeftTriggerAxis() > 0.5 ) {
        System.out.println("trigger value: " + driver.getLeftTriggerAxis());
        RobotContainer.superstructure.requestIdle();
      }
      // System.out.println("running");
      // if (RobotContainer.operator.getRightBumperPressed()) {
      //   RobotContainer.superstructure.requestIntakeConeDoubleSubstation();
      // }
  
      // if (RobotContainer.operator.getLeftBumperPressed()) {
      //   RobotContainer.superstructure.requestIntakeCubeDoubleSubstation();
      // }
  
      // if (RobotContainer.operator.getAButtonPressed()) {
      //   RobotContainer.superstructure.requestPreScore(Level.HIGH, GamePiece.CONE);
      // }
  
      // if (RobotContainer.operator.getBButtonPressed()) {
      //   RobotContainer.superstructure.requestPreScore(Level.MID, GamePiece.CONE);
      // }

      // if (RobotContainer.operator.getXButtonPressed()) {
      //   RobotContainer.superstructure.requestPreScore(Level.HIGH, GamePiece.CUBE);
      // }
  
      // if (RobotContainer.operator.getYButtonPressed()) {
      //   RobotContainer.superstructure.requestPreScore(Level.MID, GamePiece.CUBE);
      // }
  
      // if (RobotContainer.operator.getRightStickButtonPressed()) {
      //   RobotContainer.superstructure.requestScore();
      // }
  
      // if (RobotContainer.operator.getLeftStickButtonPressed()) {
      //   RobotContainer.superstructure.requestIdle();
      // }

      // if (RobotContainer.keyboard.getRawButtonPressed(1) ) {
      //   System.out.println("button 1 pressed");
      //   RobotContainer.superstructure.requestIntakeConeDoubleSubstation();
      // }

      // if (RobotContainer.keyboard.getRawButtonPressed(2) ) {
      //   System.out.println("button 2 pressed");
      //   RobotContainer.superstructure.requestIntakeCubeDoubleSubstation();
      // }

      // if (RobotContainer.keyboard.getRawButtonPressed(3) ) {
      //   System.out.println("button 3 pressed");
      //   RobotContainer.superstructure.requestScore();
      // }

      // if (RobotContainer.keyboard.getRawButtonPressed(4) ) {
      //   System.out.println("button 4 pressed");
      //   RobotContainer.superstructure.requestPreScore(Level.HIGH, GamePiece.CONE);
      // }

      // if (RobotContainer.keyboard.getRawButtonPressed(6) ) {
      //   System.out.println("button 6 pressed");
      //   RobotContainer.superstructure.requestPreScore(Level.HIGH, GamePiece.CUBE);
      // }

      // if (RobotContainer.keyboard.getRawButtonPressed(7) ) {
      //   System.out.println("button 7 pressed");
      //   RobotContainer.superstructure.requestPreScore(Level.MID, GamePiece.CONE);
      // }

      // if (RobotContainer.keyboard.getRawButtonPressed(9) ) {
      //   System.out.println("button 9 pressed");
      //   RobotContainer.superstructure.requestPreScore(Level.MID, GamePiece.CUBE);
      // }

      // if (RobotContainer.keyboard.getRawButtonPressed(11) ) {
      //   System.out.println("button 11 pressed");
      //   RobotContainer.superstructure.requestIdle();
      // }
      
    }, superstructure));
>>>>>>> 6e2a21a (add keyboard controls for operator)

    new JoystickButton(driver, XboxController.Button.kRightBumper.value).whileTrue(new AutoPickupRoutine(
        () -> new Pose2d(fieldLength - 1.312749431033244, aprilTags.get(4).getY(), new Rotation2d(0.0)),
        (pose, time) -> new Rotation2d(0.0),
        swerve,
        superstructure));

    new JoystickButton(driver, XboxController.Button.kLeftStick.value)
        .whileTrue(new AutoPlaceCommand(swerve, superstructure));
  }

  private void configureNorthstarVision() {
    northstarVision.setDataInterfaces(poseEstimator::getLatestPose, poseEstimator::addVisionData);
  }

  public Command getAutonomousCommand() {
    return new TwoPieceBalanceMode(superstructure, swerve);
  }
}

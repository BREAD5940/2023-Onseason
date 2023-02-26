package frc.robot.subsystems.vision.limelight;

import static frc.robot.Constants.LimelightVision.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightVision extends SubsystemBase {

  /* Variables */
  private final NetworkTable limelight = NetworkTableInstance
    .getDefault()
    .getTable("limelight");
  private final NetworkTableEntry tx = limelight.getEntry("tx");
  private final NetworkTableEntry ty = limelight.getEntry("ty");
  private final NetworkTableEntry ta = limelight.getEntry("ta");
  private final NetworkTableEntry tv = limelight.getEntry("tv");

  private final NetworkTableEntry cameraMode = limelight.getEntry("camMode");
  private final NetworkTableEntry ledMode = limelight.getEntry("ledMode");
  private final NetworkTableEntry pipeline = limelight.getEntry("pipeline");

  public void periodic() {
    SmartDashboard.putBoolean("LimelightHasTarget", hasTarget());

    if (!hasTarget()) {
      return;
    } else {
      SmartDashboard.putNumber("LimelightX", tx.getDouble(0.0));
      SmartDashboard.putNumber("LimelightY", ty.getDouble(0.0));
      SmartDashboard.putNumber("LimelightArea", ta.getDouble(0.0));
      SmartDashboard.putNumber("LimelightDistance", getDistance());
    }
  }

  /**
   * Gets the distance from the robot to the target
   *
   * @return the distance in inches
   */
  public double getDistance() {
    double angleToGoalDegrees = CAMERA_PITCH_DEGREES + ty.getDouble(0.0);
    double angleToTargetRadians = angleToGoalDegrees * (Math.PI / 180.0);

    double distance =
      (TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) /
      Math.tan(angleToTargetRadians);

    return distance;
  }

  /**
   * Checks if the camera is currently detecting a target
   *
   * @return true if there is a target detected, false otherwise
   */
  public boolean hasTarget() {
    double hasTarget = tv.getDouble(0.0);
    double cameraMode = getCameraMode();
    double ledMode = getLedMode();

    return (cameraMode == 0.0 && hasTarget == 1.0 && ledMode == 0.0);
  }

  /* Enables the limelight */
  public void enable() {
    setCameraMode(0);
    setLedMode(3);
    setPipeline(0);
  }

  /* Disables the limelight */
  public void disable() {
    setCameraMode(0);
    setLedMode(1);
    setPipeline(0);
  }

  /**
   * Sets the camera mode
   *
   * @param mode the camera mode to set (0 for vision processing, 1 for driver camera)
   */
  public void setCameraMode(double mode) {
    cameraMode.setNumber(mode);
  }

  /**
   * Gets the current camera mode
   *
   * @return the current camera mode
   */
  public double getCameraMode() {
    return cameraMode.getDouble(0.0);
  }

  /**
   * Sets the LED mode of the camera
   *
   * @param mode the LED mode to set (0 for default, 1 for off, 1 for blink, 3.0 for on)
   */
  public void setLedMode(double mode) {
    ledMode.setNumber(mode);
  }

  /**
   * Gets the current LED mode of the Limelight camera
   *
   * @return the current LED mode
   */
  public double getLedMode() {
    return ledMode.getDouble(0.0);
  }

  /**
   * Sets the pipeline of the camera
   *
   * @param mode the pipeline to set (0 to 9)
   */
  public void setPipeline(double mode) {
    pipeline.setNumber(mode);
  }

  /**
   * Gets the current pipeline of the Limelight camera
   *
   * @return the current pipeline
   */
  public double getPipeline() {
    return pipeline.getDouble(0.0);
  }

  /**
   * Gets the NetworkTable instance for the Limelight
   *
   * @return the NetworkTable for Limelight
   */
  public NetworkTable getLimelightTable() {
    return limelight;
  }
}

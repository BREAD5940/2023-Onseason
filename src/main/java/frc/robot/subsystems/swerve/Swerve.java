package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadUtil;

import static frc.robot.Constants.Drive.*;

public class Swerve extends SubsystemBase {

    // Hardware, kinematics, and odometry
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final ModuleIO[] moduleIOs = new ModuleIO[4];
    private final ModuleIOInputsAutoLogged[] moduleInputs = {
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged()
    };

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(FL_LOCATION, FR_LOCATION, BL_LOCATION,
            BR_LOCATION);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getRotation2d(),
            getSwerveModulePositions());
    private Pose2d poseRaw = new Pose2d();
    private double[] lastModulePositionsMeters = new double[] { 0.0, 0.0, 0.0, 0.0 };
    private Rotation2d lastGyroYaw = new Rotation2d();

    // State Variables
    private ChassisSpeeds robotSetpoints = new ChassisSpeeds(0, 0, 0);
    private SwerveState systemState = SwerveState.PERCENT;
    private boolean requestVelocity = false;
    private boolean requestPercent = false;
    private boolean fieldRelative = true;
    private boolean auto = false;
    private double lastFPGATimestamp = 0.0;

    /* Swerve States Enum */
    enum SwerveState {
        VELOCITY,
        PERCENT
    }

    /** Constructs a new swerve object */
    public Swerve() {
        gyroIO = new GyroIOPigeon2();

        moduleIOs[0] = new ModuleIOTalonFX(DRIVE_IDS[0], STEER_IDS[0], AZIMUTH_CHANNELS[0], AZIMUTH_OFFSETS[0],
                DRIVE_INVERT_TYPES[0], STEER_INVERT_TYPES[0], AZIMUTHS_ARE_REVERSED[0], "FL");
        moduleIOs[1] = new ModuleIOTalonFX(DRIVE_IDS[1], STEER_IDS[1], AZIMUTH_CHANNELS[1], AZIMUTH_OFFSETS[1],
                DRIVE_INVERT_TYPES[1], STEER_INVERT_TYPES[1], AZIMUTHS_ARE_REVERSED[1], "FR");
        moduleIOs[2] = new ModuleIOTalonFX(DRIVE_IDS[2], STEER_IDS[2], AZIMUTH_CHANNELS[2], AZIMUTH_OFFSETS[2],
                DRIVE_INVERT_TYPES[2], STEER_INVERT_TYPES[2], AZIMUTHS_ARE_REVERSED[2], "BL");
        moduleIOs[3] = new ModuleIOTalonFX(DRIVE_IDS[3], STEER_IDS[3], AZIMUTH_CHANNELS[3], AZIMUTH_OFFSETS[3],
                DRIVE_INVERT_TYPES[3], STEER_INVERT_TYPES[3], AZIMUTHS_ARE_REVERSED[3], "BR");

        for (int i = 0; i < 4; i++) {
            moduleIOs[i].setDriveBrakeMode(true);
            moduleIOs[i].setTurnBrakeMode(false);
        }
    }

    @Override
    public void periodic() {
        /** Update inputs */
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Swerve/Gyro", gyroInputs);
        for (int i = 0; i < 4; i++) {
            moduleIOs[i].updateInputs(moduleInputs[i]);
            Logger.getInstance().processInputs("Swerve/Module" + Integer.toString(i), moduleInputs[i]);
        }
        Logger.getInstance().recordOutput("Swerve/loopCycleTime",
                Logger.getInstance().getRealTimestamp() / 1.0E6 - lastFPGATimestamp);
        lastFPGATimestamp = Logger.getInstance().getRealTimestamp() / 1.0E6;

        /** Generate module setpoints */
        Rotation2d[] turnPositions = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            turnPositions[i] = new Rotation2d(moduleInputs[i].moduleAngleRads);
        }

        SwerveModuleState[] setpointStates;
        if (fieldRelative) {
            setpointStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
                    robotSetpoints.vxMetersPerSecond,
                    robotSetpoints.vyMetersPerSecond,
                    robotSetpoints.omegaRadiansPerSecond,
                    RobotContainer.poseEstimator.getLatestPose().getRotation()));
        } else {
            setpointStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(
                    robotSetpoints.vxMetersPerSecond,
                    robotSetpoints.vyMetersPerSecond,
                    robotSetpoints.omegaRadiansPerSecond));
        }

        SwerveModuleState[] setpointStatesOptimized = new SwerveModuleState[] { null, null, null, null };
        double[] desiredAngles = { 0, 0, 0, 0 };
        double[] desiredVelocities = { 0, 0, 0, 0 };

        for (int i = 0; i < 4; i++) {
            setpointStatesOptimized[i] = SwerveModuleState.optimize(setpointStates[i], turnPositions[i]);
        }

        /** Handle State Machine Things */
        SwerveState nextSystemState = systemState;

        if (systemState == SwerveState.PERCENT) {
            SwerveDriveKinematics.desaturateWheelSpeeds(setpointStatesOptimized, 1.0);

            for (int i = 0; i < 4; i++) {
                double[] desiredState = getContinousOutput(setpointStatesOptimized[i],
                        moduleInputs[i].moduleAngleRads);
                moduleIOs[i].setDrivePercent(desiredState[0]);
                moduleIOs[i].setTurnAngle(Units.radiansToDegrees(desiredState[1]));
                desiredVelocities[i] = desiredState[0];
                desiredAngles[i] = desiredState[1];
            }

            if (requestVelocity) {
                nextSystemState = SwerveState.VELOCITY;
            }
        } else if (systemState == SwerveState.VELOCITY) {
            SwerveDriveKinematics.desaturateWheelSpeeds(setpointStatesOptimized, ROBOT_MAX_SPEED);

            for (int i = 0; i < 4; i++) {
                double[] desiredState = getContinousOutput(setpointStatesOptimized[i],
                        moduleInputs[i].moduleAngleRads);
                moduleIOs[i].setDriveVelocity(desiredState[0], auto);
                moduleIOs[i].setTurnAngle(Units.radiansToDegrees(desiredState[1]));
                desiredVelocities[i] = desiredState[0];
                desiredAngles[i] = desiredState[1];
            }

            if (requestPercent) {
                nextSystemState = SwerveState.PERCENT;
            }
        }
        systemState = nextSystemState;

        /** Additional Logging */
        SwerveModuleState[] measuredStates = getMeasuredStates();
        logModuleStates("SwerveModuleStates/Setpoints", setpointStates);
        logModuleStates("SwerveModuleStates/SetpointsOptimized", setpointStatesOptimized);
        logModuleStates("SwerveModuleStates/MeasuredStates", measuredStates);
        updateOdometry();
        Logger.getInstance().recordOutput("Desired Angles", desiredAngles);
        Logger.getInstance().recordOutput("Desired Velocities", desiredVelocities);
        Logger.getInstance().recordOutput("Swerve Velocity (mag.)", getVelocity().getNorm());
        Logger.getInstance().recordOutput("Swerve X Velocity",
                getVelocity().rotateBy(RobotContainer.poseEstimator.getLatestPose().getRotation()).getX());
        Logger.getInstance().recordOutput("Swerve Y Velocity",
                getVelocity().rotateBy(RobotContainer.poseEstimator.getLatestPose().getRotation()).getY());
        Logger.getInstance().recordOutput("Swerve Setpoints/Swerve Desired dX", robotSetpoints.vxMetersPerSecond);
        Logger.getInstance().recordOutput("Swerve Setpoints/Swerve Desired dY", robotSetpoints.vyMetersPerSecond);
        Logger.getInstance().recordOutput("Swerve Setpoints/Swerve Desired Omega",
                robotSetpoints.omegaRadiansPerSecond);

        Logger.getInstance().recordOutput("Field-Relative Swerve Velocity (angle)",
                getVelocity().rotateBy(getRotation2d()).getAngle().getDegrees());
        Logger.getInstance().recordOutput("Robot-Relative Swerve Velocity (angle)",
                getVelocity().getAngle().getDegrees());
        Logger.getInstance().recordOutput("Odometry/PoseRaw", poseRaw);

    }

    /** Requests a provided percent output to the swerve drive */
    public void requestPercent(ChassisSpeeds speeds, boolean fieldRelative) {
        if (Math.abs(speeds.vxMetersPerSecond) > 0.05 || Math.abs(speeds.vyMetersPerSecond) > 0.05
                || Math.abs(speeds.omegaRadiansPerSecond) > 0.05) {
            robotSetpoints = speeds;
        } else {
            robotSetpoints = new ChassisSpeeds(0, 0, 0);
        }
        this.fieldRelative = fieldRelative;
        this.requestPercent = true;
        this.requestVelocity = false;
    }

    /** Requests a provided velocity to the swerve drive */
    public void requestVelocity(ChassisSpeeds speeds, boolean fieldRelative, boolean auto) {
        robotSetpoints = speeds;
        this.fieldRelative = fieldRelative;
        this.requestPercent = false;
        this.requestVelocity = true;
        this.auto = auto;
    }

    /** Updates the swerve drive odometry */
    public void updateOdometry() {
        var twist = kinematics.toTwist2d(getDeltaSwerveModulePositions());

        var gyroYaw = new Rotation2d(gyroInputs.positionRad);
        if (gyroInputs.connected) {
            twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());
        }
        lastGyroYaw = gyroYaw;

        RobotContainer.poseEstimator.addDriveData(
                Timer.getFPGATimestamp(),
                twist);

        poseRaw = odometry.update(
                getRotation2d(),
                getSwerveModulePositions());
    }

    /** Sets the neutral modes for the swerve modules */
    public void setNeutralModes(NeutralMode mode) {
        for (int i = 0; i < 4; i++) {
            moduleIOs[i].setDriveBrakeMode(mode == NeutralMode.Brake ? true : false);
        }
    }

    /** Returns rotation2d with angle of robot */
    public Rotation2d getRotation2d() {
        return new Rotation2d(gyroInputs.positionRad);
    }

    /** Returns the raw gyro angle */
    public double getGyroDegrees() {
        return gyroInputs.positionDegRaw;
    }

    /** Resets raw */
    public void resetRaw() {
        odometry.resetPosition(getRotation2d(), getSwerveModulePositions(), RobotContainer.poseEstimator.getLatestPose());
        poseRaw = odometry.getPoseMeters();
    }

    /** Returns the raw pose of the robot */
    public Pose2d getRawPose() {
        return poseRaw;
    }

    /** Returns the measured states of the swerve drive */
    public SwerveModuleState[] getMeasuredStates() {
        SwerveModuleState[] measuredStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measuredStates[i] = new SwerveModuleState(
                    moduleInputs[i].driveVelocityMetersPerSec,
                    new Rotation2d(moduleInputs[i].moduleAngleRads));
        }
        return measuredStates;
    }

    /** Returns the Swerve Modules' positions */
    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = new SwerveModulePosition(
                    moduleInputs[i].driveDistanceMeters,
                    new Rotation2d(moduleInputs[i].moduleAngleRads));
        }
        return modulePositions;
    }

    /** Returns the roll of the swerve's gyro in radians */
    public double getRoll() {
        return gyroInputs.rollRad;
    }

    /** Returns the pitch of the swerve's gyro in radians */
    public double getPitch() {
        return gyroInputs.pitchRad;
    }

    public SwerveModulePosition[] getDeltaSwerveModulePositions() {
        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            wheelDeltas[i] = new SwerveModulePosition(
                    (moduleInputs[i].driveDistanceMeters - lastModulePositionsMeters[i]),
                    new Rotation2d(moduleInputs[i].moduleAngleRads));

            lastModulePositionsMeters[i] = moduleInputs[i].driveDistanceMeters;
        }

        return wheelDeltas;
    }

    /** Returns the ROBOT RELATIVE velocity of the robot */
    public Translation2d getVelocity() {
        SwerveModuleState[] measuredStates = getMeasuredStates();
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(
                measuredStates[0],
                measuredStates[1],
                measuredStates[2],
                measuredStates[3]);
        return new Translation2d(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond);
    }

    /** Returns continous output */
    private double[] getContinousOutput(SwerveModuleState desiredState, double currentAngle) {
        double absoluteHeading = currentAngle % (2.0 * Math.PI);
        if (absoluteHeading < 0.0) {
            absoluteHeading += 2.0 * Math.PI;
        }
        double adjustedDesiredAngle = BreadUtil.getRadians0To2PI(desiredState.angle) + currentAngle - absoluteHeading;
        if (BreadUtil.getRadians0To2PI(desiredState.angle) - absoluteHeading > Math.PI) {
            return new double[] {
                    desiredState.speedMetersPerSecond,
                    adjustedDesiredAngle - 2.0 * Math.PI
            };
        } else if (BreadUtil.getRadians0To2PI(desiredState.angle) - absoluteHeading < -Math.PI) {
            return new double[] {
                    desiredState.speedMetersPerSecond,
                    adjustedDesiredAngle + 2.0 * Math.PI
            };
        } else {
            return new double[] {
                    desiredState.speedMetersPerSecond,
                    adjustedDesiredAngle
            };
        }
    }

    /** Method used to log module states */
    private void logModuleStates(String key, SwerveModuleState[] states) {
        List<Double> dataArray = new ArrayList<Double>();
        for (int i = 0; i < 4; i++) {
            dataArray.add(states[i].angle.getRadians());
            dataArray.add(states[i].speedMetersPerSecond);
        }
        Logger.getInstance().recordOutput(key, dataArray.stream().mapToDouble(Double::doubleValue).toArray());
    }

    /** Resets the integrated sensor on the steer motors for all swerve modules */
    public void resetAllToAbsolute() {
        for (int i = 0; i < 4; i++) {
            moduleIOs[i].resetToAbsolute();
        }
    }

}

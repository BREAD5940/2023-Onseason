package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commons.BreadPoseEstimator;
import frc.robot.commons.BreadUtil;
import frc.robot.subsystems.vision.Vision.TimestampedPose2d;

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

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(FL_LOCATION, FR_LOCATION, BL_LOCATION, BR_LOCATION);
    private final BreadPoseEstimator poseEstimator =
        new BreadPoseEstimator(
            kinematics,
            getRotation2d(),
            getSwerveModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(0.8)),
            VecBuilder.fill(0.6, 0.6, Units.degreesToRadians(40.0)));

    private Pose2d pose = new Pose2d(); 
    public final Field2d field = new Field2d();

    // State Variables
    private ChassisSpeeds robotSetpoints = new ChassisSpeeds(0, 0, 0);
    private SwerveState systemState = SwerveState.PERCENT;
    private boolean requestVelocity = false;
    private boolean requestPercent = false;
    private boolean fieldRelative = true;

    /* Swerve States Enum */
    enum SwerveState {
        VELOCITY, 
        PERCENT
    }


    /** Constructs a new swerve object */
    public Swerve() {
        gyroIO = new GyroIOPigeon2();

            moduleIOs[0] = new ModuleIOTalonFX(DRIVE_IDS[0], STEER_IDS[0], AZIMUTH_CHANNELS[0], AZIMUTH_OFFSETS[0], DRIVE_INVERT_TYPES[0], STEER_INVERT_TYPES[0], AZIMUTHS_ARE_REVERSED[0], "FL");
            moduleIOs[1] = new ModuleIOTalonFX(DRIVE_IDS[1], STEER_IDS[1], AZIMUTH_CHANNELS[1], AZIMUTH_OFFSETS[1], DRIVE_INVERT_TYPES[1], STEER_INVERT_TYPES[1], AZIMUTHS_ARE_REVERSED[1], "FR");
            moduleIOs[2] = new ModuleIOTalonFX(DRIVE_IDS[2], STEER_IDS[2], AZIMUTH_CHANNELS[2], AZIMUTH_OFFSETS[2], DRIVE_INVERT_TYPES[2], STEER_INVERT_TYPES[2], AZIMUTHS_ARE_REVERSED[2], "BL");
            moduleIOs[3] = new ModuleIOTalonFX(DRIVE_IDS[3], STEER_IDS[3], AZIMUTH_CHANNELS[3], AZIMUTH_OFFSETS[3], DRIVE_INVERT_TYPES[3], STEER_INVERT_TYPES[3], AZIMUTHS_ARE_REVERSED[3], "BR");

        field.setRobotPose(pose);

        for (int i = 0; i < 4; i++) {
            moduleIOs[i].setDriveBrakeMode(true);
            moduleIOs[i].setTurnBrakeMode(true);
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
        Logger.getInstance().recordOutput("Odometry", getPose());

        /** Generate module setpoints */
        Rotation2d[] turnPositions = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            turnPositions[i] = new Rotation2d(moduleInputs[i].turnAbsolutePositionRad);
        }

        SwerveModuleState[] setpointStates;
        if (fieldRelative) {
            setpointStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
                robotSetpoints.vxMetersPerSecond, 
                robotSetpoints.vyMetersPerSecond, 
                robotSetpoints.omegaRadiansPerSecond, 
                new Rotation2d(gyroInputs.positionRad)
            ));
        } else {
            setpointStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(
                robotSetpoints.vxMetersPerSecond, 
                robotSetpoints.vyMetersPerSecond, 
                robotSetpoints.omegaRadiansPerSecond
            ));
        }

        SwerveModuleState[] setpointStatesOptimized = new SwerveModuleState[] {null, null, null, null};
        double[] desiredAngles = {0, 0, 0, 0};
        double[] desiredVelocities = {0, 0, 0, 0};

        for (int i = 0; i < 4; i++) {
            setpointStatesOptimized[i] = SwerveModuleState.optimize(setpointStates[i], turnPositions[i]);
        }

        /** Handle State Machine Things */
        SwerveState nextSystemState = systemState;

        if (systemState == SwerveState.PERCENT) {
            SwerveDriveKinematics.desaturateWheelSpeeds(setpointStatesOptimized, 1.0);

            for (int i = 0; i < 4; i++) {
                double[] desiredState = getContinousOutput(setpointStatesOptimized[i], moduleInputs[i].turnAbsolutePositionRad);
                moduleIOs[i].setDrivePercent(desiredState[0]);
                moduleIOs[i].setTurnAngle(desiredState[1]);
                desiredVelocities[i] = desiredState[0];
                desiredAngles[i] = desiredState[1];
            }

            if (requestVelocity) {
                nextSystemState = SwerveState.VELOCITY;
            }
        } else if (systemState == SwerveState.VELOCITY) {
            SwerveDriveKinematics.desaturateWheelSpeeds(setpointStatesOptimized, ROBOT_MAX_SPEED);

            for (int i = 0; i < 4; i++) {
                double[] desiredState = getContinousOutput(setpointStatesOptimized[i], moduleInputs[i].turnAbsolutePositionRad);
                moduleIOs[i].setDriveVelocity(desiredState[0]);
                moduleIOs[i].setTurnAngle(desiredState[1]);
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
        Logger.getInstance().recordOutput("Odometry/Robot", pose);
        Logger.getInstance().recordOutput("Desired Angles", desiredAngles);
        Logger.getInstance().recordOutput("Desired Velocities", desiredVelocities);
        Logger.getInstance().recordOutput("Swerve Velocity (mag.)", getVelocity().getNorm());
        Logger.getInstance().recordOutput("Field-Relative Swerve Velocity (angle)", getVelocity().rotateBy(getRotation2d()).getAngle().getDegrees());
        Logger.getInstance().recordOutput("Robot-Relative Swerve Velocity (angle)", getVelocity().getAngle().getDegrees());

    }

    /** Requests a provided percent output to the swerve drive */
    public void requestPercent(ChassisSpeeds speeds, boolean fieldRelative) {
        if (Math.abs(speeds.vxMetersPerSecond) > 0.05 || Math.abs(speeds.vyMetersPerSecond) > 0.05 || Math.abs(speeds.omegaRadiansPerSecond) > 0.05) {
            robotSetpoints = speeds;
        } else {
            robotSetpoints = new ChassisSpeeds(0, 0, 0);
        }
        this.fieldRelative = fieldRelative;
        this.requestPercent = true;
        this.requestVelocity = false;
    }

    /** Requests a provided velocity to the swerve drive */
    public void requestVelocity(ChassisSpeeds speeds, boolean fieldRelative) {
        if (Math.abs(speeds.vxMetersPerSecond) > 0.05 || Math.abs(speeds.vyMetersPerSecond) > 0.05 || Math.abs(speeds.omegaRadiansPerSecond) > 0.05) {
            robotSetpoints = speeds;
        } else {
            robotSetpoints = new ChassisSpeeds(0, 0, 0);
        }
        this.fieldRelative = fieldRelative;
        this.requestPercent = false;
        this.requestVelocity = true;
    }

    /** Resets the swerve drive odometry */
    public void reset(Pose2d newPose) {
        poseEstimator.resetPosition(getRotation2d(), getSwerveModulePositions(), newPose);
        pose = poseEstimator.getEstimatedPosition();
        gyroIO.reset();
    }

    /** Updates the swerve drive odometry */
    public void updateOdometry() {
        pose = poseEstimator.update(
            getRotation2d(),
            getSwerveModulePositions()
        );  

        while (RobotContainer.vision.hasMeasurements()) {
            TimestampedPose2d result = RobotContainer.vision.popMeasurement();
            double timestamp = result.timestamp();
            Pose2d pose = result.pose();
            poseEstimator.addVisionMeasurement(pose, timestamp);
            Pose2d bufferPose = poseEstimator.getPoseAtTime(timestamp);
            Logger.getInstance().recordOutput("Odometry/VisionErrorXInches",  Units.metersToInches(bufferPose.getX()-pose.getX()));
            Logger.getInstance().recordOutput("Odometry/VisionErrorYInches",  Units.metersToInches(bufferPose.getY()-pose.getY()));
            Logger.getInstance().recordOutput("Odometry/VisionErrorRotationDeg",  bufferPose.getRotation().getDegrees()-pose.getRotation().getDegrees());
            Logger.getInstance().recordOutput("Odometry/VisionRobot", pose);
        }
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

    /** Returns the measured states of the swerve drive */
    public SwerveModuleState[] getMeasuredStates() {
        SwerveModuleState[] measuredStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            measuredStates[i] = new SwerveModuleState(
                moduleInputs[i].driveVelocityMetersPerSec, 
                new Rotation2d(moduleInputs[i].turnAbsolutePositionRad)
            );
        }
        return measuredStates;
    }

    /** Returns the Swerve Modules' positions */
    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = new SwerveModulePosition(
                moduleInputs[i].driveDistance,
                new Rotation2d(moduleInputs[i].turnAbsolutePositionRad)
            );
        }
        return modulePositions;
    }

    /** Returns the ROBOT RELATIVE velocity of the robot */
    public Translation2d getVelocity() {
        SwerveModuleState[] measuredStates = getMeasuredStates();
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(
            measuredStates[0],
            measuredStates[1],
            measuredStates[2],
            measuredStates[3]
        );
        return new Translation2d(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond
        );
    }

    /** Returns the pose of the robot */
    public Pose2d getPose() {
        return pose;
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
    
}

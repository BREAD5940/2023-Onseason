// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import Jama.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import static frc.robot.FieldConstants.*;

// All Constants
public final class Constants {

    public static boolean tuningMode = false;
    
    // Constants pertaining to the drive subsystem go here
    public static class Drive {
    
        // Motor IDs 
        public static final int[] DRIVE_IDS = {1, 2, 3, 4};
        public static final int[] STEER_IDS = {5, 6, 7, 8};
        public static final int PIGEON_ID = 30;
        public static final int HORIZONTAL_GUT = 10;
        public static final int VERTICAL_GUT = 11;

        // Encoder Channels
        public static final int[] AZIMUTH_CHANNELS = {21, 22, 23, 24};

        // Reversed Constants
        public static final TalonFXInvertType[] DRIVE_INVERT_TYPES = {
            TalonFXInvertType.CounterClockwise, 
            TalonFXInvertType.Clockwise, 
            TalonFXInvertType.CounterClockwise, 
            TalonFXInvertType.Clockwise
        };
        // public static final boolean[] STEERS_ARE_REVERSED = {false, false, false, false};
        public static final TalonFXInvertType[] STEER_INVERT_TYPES = {
            TalonFXInvertType.Clockwise,
            TalonFXInvertType.Clockwise,
            TalonFXInvertType.Clockwise,
            TalonFXInvertType.Clockwise
        };

        public static final boolean[] AZIMUTHS_ARE_REVERSED = {false, false, false, false};
        
        /* Procedure for setting CANCoder Offsets:
         * 
         * NOTE: Azimuth offsets only effect the measured angle once
         * the sensor has been power cycled!
         * 
         * NOTE: Azimuth offsets need to be the inverse of the 
         * measured angle to correctly null-out the CANCoder absolute
         * angle
         * 
         * 1. Set all of the offsets below to 0 degrees
         * 2. Deploy the code to the robot and wait for the 
         * "[pheonix] Library initializaiton is complete message"
         * 3. Power cycle the CANCoders
         * 4. For each CANCoder, check the integrated sensor position in the 
         * "self-test snapshot" tab. Verify that the offset is 0.0
         * in the config tab. 
         * 5. Set the offsets below to the inverse angle (sign swapped)
         * of what you read below. 
         * 6. Re-deploy code to the robot and power cycle.
         */

        // public static final Rotation2d[] AZIMUTH_OFFSETS = {
        //     Rotation2d.fromDegrees(-82.178), // FL
        //     Rotation2d.fromDegrees(-114.697), // FR
        //     Rotation2d.fromDegrees(47.021), // BL
        //     Rotation2d.fromDegrees(-37.617) //BR
        // };

        public static final Rotation2d[] AZIMUTH_OFFSETS = {
            Rotation2d.fromDegrees(67.5), // FL
            Rotation2d.fromDegrees(259.1), // FR
            Rotation2d.fromDegrees(57.56), // BL
            Rotation2d.fromDegrees(65.478515625) //BR
        };

        // Drive-by shooting constants
        public static final double TANGENTIAL_SHOT_SCALAR = 0.6;
        public static final double RADIAL_SHOT_SCALAR = 0.9;

        // Measurements/Gearings
        public static final double DRIVE_GEARING = 1/6.75;
        public static final double STEER_GEARING = (24.0/8) * (72.0/14);
        public static final double ROBOT_WIDTH = Units.inchesToMeters(27.0 - 2.625 * 2.0);
        public static final double ROBOT_LENGTH = Units.inchesToMeters(28.0 - 2.625 * 2.0);
        // Madtown field callibration constant factor is 0.97
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0) * 0.9442667069;
        public static final Translation2d FIELD_TO_TARGET = new Translation2d(Units.feetToMeters(27), Units.feetToMeters(13.5));
        public static final double CAMERA_TO_SHOOTER_DISTANCE = Units.inchesToMeters(15.0);
        public static final double UPPER_HUB_RADIUS = Units.inchesToMeters(53.38)/2;
        public static final double ROBOT_MAX_SPEED = (6380.0 * DRIVE_GEARING * 2.0 * Math.PI * WHEEL_RADIUS) / 60.0;
        public static final Translation2d FL_LOCATION = new Translation2d(ROBOT_LENGTH/2, ROBOT_WIDTH/2);
        public static final Translation2d FR_LOCATION = new Translation2d(ROBOT_LENGTH/2, -ROBOT_WIDTH/2);
        public static final Translation2d BL_LOCATION = new Translation2d(-ROBOT_LENGTH/2, ROBOT_WIDTH/2);
        public static final Translation2d BR_LOCATION = new Translation2d(-ROBOT_LENGTH/2, -ROBOT_WIDTH/2); 
    
        // Other
        public static final double CANCODER_RESOLUTION = 4096.0;

    }

    // Constants pertaining to the elevator subsystem go here 
    public static class Elevator {
        public static final int ELEVATOR_LEFT_ID = 11;
        public static final int ELEVATOR_RIGHT_ID = 12;

        public static final double ELEVATOR_GEARING = (9.0/44.0);
        public static final double ELEVATOR_PULLEY_PITCH_DIAMETER = 0.06096;

        public static final double ELEVATOR_BELOW_STAGE1_KG = -0.004;
        public static final double ELEVATOR_ABOVE_STAGE1_KG = 0.028;

        public static final double SECOND_STAGE_HEIGHT = 0.5872286463821073;

        public static final double ELEVATOR_HOMING_POSITION = Units.inchesToMeters(1.0);

        public static final double ELEVATOR_MIN_LIMITED_ARM_ROM = 0.18;
        public static final double ELEVATOR_MIN = Units.inchesToMeters(-5.0);
        public static final double ELEVATOR_MAX = 1.248288847926422;
        public static final double ELEVATOR_SETPOINT_TOLERANCE = Units.inchesToMeters(3.0);
        public static final double ELEVATOR_MAX_VELOCITY = (6380.0 * (10.5/12.0) * ELEVATOR_GEARING * ELEVATOR_PULLEY_PITCH_DIAMETER * Math.PI)/60.0;

        public static final double ELEVATOR_IDLE_POSE = 0.18;
        public static final double ELEVATOR_PRE_CUBE_HIGH = 0.85;
        public static final double ELEVATOR_PRE_CONE_HIGH = 1.21;
        public static final double ELEVATOR_CONE_SLAM_HIGH = 1.21;
        public static final double ELEVATOR_CONE_PULL_OUT_HIGH = 1.2;
        public static final double ELEVATOR_CUBE_OFFSET = 0.509856;
        public static final double ELEVATOR_CONE_OFFSET = 0.54;
        public static final double ELEVATOR_PRE_LOW = 0.34;
        public static final double ELEVATOR_FLOOR_INTAKE_CUBE = 0.15;
        public static final double ELEVATOR_DOUBLE_SUBSTATION_CONE = 1.15;
        // public static final double ELEVATOR_DOUBLE_SUBSTATION_CUBE = 1.115;
        public static final double ELEVATOR_DOUBLE_SUBSTATION_CUBE = 0.19550697818942656;

        public static final TalonFXInvertType ELEVATOR_LEFT_INVERT_TYPE = TalonFXInvertType.CounterClockwise;
        public static final TalonFXInvertType ELEVATOR_RIGHT_INVERT_TYPE = TalonFXInvertType.CounterClockwise;
    }

    // Constants pertaining to the arm subsystem go here
    public static class Arm {
        public static final int ARM_ID = 14; 
        public static final TalonFXInvertType ARM_INVERT_TYPE = TalonFXInvertType.Clockwise;
        public static final Rotation2d ARM_ENCODER_OFFSET = Rotation2d.fromDegrees(0.0);

        public static final int ARM_AZIMUTH_ID = 31;
        public static final boolean ARM_AZIMUTH_INVERTED = true;
        public static final double ARM_AZIMUTH_DEGREE_OFFSET = 312.451171875 - 90.0; // Subtract 90 from the offset because the zero was obtained when the arm was pointing straight up
        public static final double ARM_GEAR_RATIO = (1.0/78.7) * (60.0/27.0);
        public static final double ARM_MAX_VELOCITY = ((1.0/78.7) * 6380.0 * 360.0)/60.0;

        public static final double ARM_MAX_LIMITED_ELEVATOR_ROM = 117.24609375; 
        public static final double ARM_NEUTRAL_ANGLE = 110.0;
        public static final double ARM_MIN = 8.61328125;
        public static final double ARM_MAX = 236.77734375;
        public static final double ARM_SETPOINT_TOLERANCE = 3.0;
        public static final double ARM_NULL_RANGE = 310.0; // To 360 degrees

        public static final double ARM_IDLE_POSE = 90.0;
        public static final double ARM_PRE_SCORE_CUBE = 169.0;
        public static final double ARM_PRE_SCORE_CONE = 118.076;
        public static final double ARM_SLAM_CONE = 187.3828;
        public static final double ARM_PRE_SCORE_LOW = 230.0;
        public static final double ARM_FLOOR_INTAKE_CUBE = 1.0;
        public static final double ARM_DOUBLE_SUBSTATION_CONE = 178.4;
        // public static final double ARM_DOUBLE_SUBSTATION_CUBE = 211.6;
        public static final double ARM_DOUBLE_SUBSTATION_CUBE = 155.478515625;
        public static final double ARM_UNJAM_POSITION = 26.0;

        public static final double ARM_KP = 0.03;
        public static final double ARM_KI = 0.0;
        public static final double ARM_KD = 0.0003;
        public static final double ARM_EFFECTIVE_MAX_VELOCITY = 405.000000;
        public static final double ARM_KG = 0.026;
        public static final double ARM_KA = 0.0;

        public static final double ARM_MAX_MOTION_ACCELERATION = 1400.0;
        public static final double ARM_MAX_MOTION_CRUISE_VELOCITY = 375.0;
        
    }

    // Constants pertaining to the end effector subsystem go here 
    public static class EndEffector {
        public static final int MOTOR_ID = 18;
        public static final TalonFXInvertType INVERSION = TalonFXInvertType.Clockwise;

        public static final double INTAKE_CONE_CURR_LIMIT = 70.0;
        public static final double INTAKE_CUBE_CURR_LIMIT = 25.0;
    }

    // Constants pertaining to the floor intake subsystem
    public static class FloorIntake {
        public static final int DEPLOY_ID = 16;
        public static final int ROLLER_ID = 17;

        public static final double DEPLOY_GEAR_RATIO = (12.0/36) * (20.0/100) * (18.0/60) * (12.0/24);
        public static final TalonFXInvertType DEPLOY_INVERT_TYPE = TalonFXInvertType.Clockwise;
        public static final TalonFXInvertType ROLLER_INVERT_TYPE = TalonFXInvertType.CounterClockwise;
        public static final double DEPLOY_MAX_SPEED = (6380.0 * DEPLOY_GEAR_RATIO * 360.0)/60.0;
        public static final double INTAKE_MIN_POSITION = 0.0;
        public static final double INTAKE_MAX_POSITION = 190.75;
        public static final double INTAKE_IDLE_POSITION = 28.04046224348974;
        public static final double INTAKE_UNJAM_POSITION = 42.0;

        public static final double FLOOR_INTAKE_KP = 0.0;
        public static final double FLOOR_INTAKE_KD = 0.0;
        public static final double FLOOR_INTAKE_KF = 0.0;
        public static final double FLOOR_INTAKE_KG = 0.0;

        public static final double FLOOR_INTAKE_ZERO = -5.0;
    }

    // Constants pertaining to the climber subsystem
    public static class Climber {
        public static final int CLIMBER_ID = 20;
        public static final double CLIMBING_FF = 0.0;
        public static final double CLIMBER_GEARING = 1.0;
        public static final double CLIMBER_PULLEY_DIAMETER = 1.0;
        public static final double CLIMBER_DEPLOY_HEIGHT = 117.59343321852536;
        public static final double CLIMBER_SETPOINT_TOLERANCE = 3.0;
        public static final TalonFXInvertType CLIMBER_INVERT_TYPE = TalonFXInvertType.Clockwise;
    }

    // Constants pertaining to electrical
    public static class Electrical {
        public static final double FALCON_FULL_THROTTLE = 1023.0;
        public static final String CANIVORE_BUS_NAME = "dabus"; 
    }

    // Constants pertaining to the camera
    public static class Vision {
        public static final Transform3d ROBOT_TO_LL =
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(12), Units.inchesToMeters(-9), Units.inchesToMeters(10)),
                        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-23.5), 0.0)); //Units.degreesToRadians(1.6593493)));
        public static final double HIGH_TAPE_OFF_GROUND = 1.12;
        public static final double MID_TAPE_OFF_GROUND = 0.61;
        public static final double X_SCORING_POSITION = 1.85;
        public static final double AMBIGUITY_THRESHOLD = 0.15;

    }

    // Constants pertaining to the LEDs 
    public static class LEDs {
        public static final int[] PURPLE = {255, 0, 255};
        public static final int[] YELLOW = {255, 255, 0};
    }

    // Constants pertaining to robot locations on the field
    public static class RobotLocations {
        public static final Pose2d FIELD_SIDE_PICKUP_LOCATION = new Pose2d(aprilTags.get(4).getX() - Units.inchesToMeters(51.4), aprilTags.get(4).getY() - Units.inchesToMeters(29), new Rotation2d());
        public static final Pose2d WALL_SIDE_PICKUP_LOCATION = new Pose2d(aprilTags.get(4).getX() - Units.inchesToMeters(51.4), aprilTags.get(4).getY() + Units.inchesToMeters(29), new Rotation2d());

    }

    // Ips for the stuff
    public static class Network {
        public static final String radio = "10.59.40.1";
        public static final String orangepi1 = "10.59.40.30";
        public static final String orangepi2 = "10.59.40.31";
        public static final String limelight = "10.59.40.11";
    }
  
}

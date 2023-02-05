// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

// All Constants
public final class Constants {

    public static boolean tuningMode = true;
    
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

        public static final Rotation2d[] AZIMUTH_OFFSETS = {
            Rotation2d.fromDegrees(-82.178), // FL
            Rotation2d.fromDegrees(-114.697), // FR
            Rotation2d.fromDegrees(47.021), // BL
            Rotation2d.fromDegrees(-37.617) //BR
        };

        // Drive-by shooting constants
        public static final double TANGENTIAL_SHOT_SCALAR = 0.6;
        public static final double RADIAL_SHOT_SCALAR = 0.9;

        // Measurements/Gearings
        public static final double MODULE_GEARING = (14.0/50.0) * (28.0/16.0) * (15.0/45.0);
        public static final double ROBOT_WIDTH = Units.inchesToMeters(27.0);
        public static final double ROBOT_LENGTH = Units.inchesToMeters(28.0);
        // Madtown field callibration constant factor is 0.97
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0) * 0.9442667069;
        public static final Translation2d FIELD_TO_TARGET = new Translation2d(Units.feetToMeters(27), Units.feetToMeters(13.5));
        public static final double CAMERA_TO_SHOOTER_DISTANCE = Units.inchesToMeters(15.0);
        public static final double UPPER_HUB_RADIUS = Units.inchesToMeters(53.38)/2;
        public static final double ROBOT_MAX_SPEED = (6380.0 * MODULE_GEARING * 2.0 * Math.PI * WHEEL_RADIUS) / 60.0;
        public static final Translation2d FL_LOCATION = new Translation2d(ROBOT_LENGTH/2, ROBOT_WIDTH/2);
        public static final Translation2d FR_LOCATION = new Translation2d(ROBOT_LENGTH/2, -ROBOT_WIDTH/2);
        public static final Translation2d BL_LOCATION = new Translation2d(-ROBOT_LENGTH/2, ROBOT_WIDTH/2);
        public static final Translation2d BR_LOCATION = new Translation2d(-ROBOT_LENGTH/2, -ROBOT_WIDTH/2); 
    
        // Other
        public static final double CANCODER_RESOLUTION = 4096.0;

    }

    // Constantsp pertaining to the elevator subsystem go here 
    public static class Elevator {
        public static final int ELEVATOR_LEFT_ID = 11;
        public static final int ELEVATOR_RIGHT_ID = 12;

        public static final double ELEVATOR_GEARING = 0.13636363636;
        public static final double ELEVATOR_PULLEY_PITCH_DIAMETER = 0.06096;
        public static final double GRAVITY_FF = 0.00916666666;
        public static final double CF_SPRING_FF = 0.00916666666;
        public static final double SECOND_STAGE_HEIGHT = 0.5657486369247514;

        public static final double ELEVATOR_MIN_LIMITED_ARM_ROM = 0.18;
        public static final double ELEVATOR_MIN = 0.0;
        public static final double ELEVATOR_MAX = 1.1712183954944326;
        public static final double ELEVATOR_SETPOINT_TOLERANCE = Units.inchesToMeters(3.0);
        public static final double ELEVATOR_MAX_VELOCITY = (6380.0 * ELEVATOR_GEARING * ELEVATOR_PULLEY_PITCH_DIAMETER * Math.PI)/60.0;

        public static final double ELEVATOR_PRE_CUBE_HIGH = 0.857;
        public static final double ELEVATOR_PRE_CONE_HIGH = 1.115;
        public static final double ELEVATOR_CONE_SLAM_HIGH = 1.115;
        public static final double ELEVATOR_CONE_PULL_OUT_HIGH = 0.936;
        public static final double ELEVATOR_CUBE_OFFSET = 0.509856;
        public static final double ELEVATOR_CONE_OFFSET = 0.4702;
        public static final double ELEVATOR_PRE_LOW = 0.34;
        public static final double ELEVATOR_FLOOR_INTAKE_CUBE = 0.05;
        public static final double ELEVATOR_DOUBLE_SUBSTATION_CONE = 1.1;
        // public static final double ELEVATOR_DOUBLE_SUBSTATION_CUBE = 1.115;
        public static final double ELEVATOR_DOUBLE_SUBSTATION_CUBE = 0.19550697818942656;
    }

    // Constants pertaining to the arm subsystem go here
    public static class Arm {
        public static final int ARM_ID = 14; 
        public static final TalonFXInvertType ARM_INVERT_TYPE = TalonFXInvertType.Clockwise;
        public static final Rotation2d ARM_ENCODER_OFFSET = Rotation2d.fromDegrees(0.0);

        public static final int ARM_AZIMUTH_ID = 31;
        public static final double ARM_AZIMUTH_DEGREE_OFFSET = 196.875;
        public static final double ARM_GEAR_RATIO = (1.0/78.7);
        public static final double ARM_MAX_VELOCITY = ((1.0/78.7) * 6380.0 * 360.0)/60.0;

        public static final double ARM_MAX_LIMITED_ELEVATOR_ROM = 117.24609375; 
        public static final double ARM_NEUTRAL_ANGLE = 90.0;
        public static final double ARM_MIN = 0.0;
        public static final double ARM_MAX = 236.77734375;
        public static final double ARM_SETPOINT_TOLERANCE = 3.0;
        public static final double ARM_NULL_RANGE = 300.0; // To 360 degrees

        public static final double ARM_PRE_SCORE_CUBE = 185.88867;
        public static final double ARM_PRE_SCORE_CONE = 138.076;
        public static final double ARM_SLAM_CONE = 187.3828;
        public static final double ARM_PRE_SCORE_LOW = 230.0;
        public static final double ARM_FLOOR_INTAKE_CUBE = 1.0;
        public static final double ARM_DOUBLE_SUBSTATION_CONE = 178.4;
        // public static final double ARM_DOUBLE_SUBSTATION_CUBE = 211.6;
        public static final double ARM_DOUBLE_SUBSTATION_CUBE = 155.478515625;
    }

    // Constants pertaining to the end effector subsystem go here 
    public static class EndEffector {
        public static final int MOTOR_ID = 18;
        public static final boolean IS_REVERSED = false;
    }

    // Constants pertaining to electrical
    public static class Electrical {
        public static final String CANIVORE_BUS_NAME = "dabus"; 
    }
  
}

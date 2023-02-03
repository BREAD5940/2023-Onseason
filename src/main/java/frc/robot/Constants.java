// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

// All Constants
public final class Constants {
    
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
        public static final double ROBOT_WIDTH = Units.inchesToMeters(25.0);
        public static final double ROBOT_LENGTH = Units.inchesToMeters(30.0);
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
        public static final double ELEVATOR_PULLEY_PITCH_DIAMETER = 0.028575;
        public static final double GRAVITY_FF = 0.00907380849;

        public static final double ELEVATOR_MIN_LIMITED_ARM_ROM = 0.5;
        public static final double ELEVATOR_MIN = 0.0;
        public static final double ELEVATOR_MAX = 1.0;
        public static final double ELEVATOR_SETPOINT_TOLERANCE = Units.inchesToMeters(3.0);
    }

    // Constants pertaining to the arm subsystem go here
    public static class Arm {
        public static final int ARM_ID = 14; 
        public static final TalonFXInvertType ARM_INVERT_TYPE = TalonFXInvertType.Clockwise;
        public static final Rotation2d ARM_ENCODER_OFFSET = Rotation2d.fromDegrees(0.0);

        public static final int ARM_AZIMUTH_ID = 31;
        public static final double ARM_AZIMUTH_DEGREE_OFFSET = 196.875;
        public static final double ARM_GEAR_RATIO = (1.0/78.7);
        public static final double ARM_MAX_VELOCITY = ((1.0/78.7) * 6380.0 * 2 * Math.PI)/60.0;

        public static final double ARM_MAX_LIMITED_ELEVATOR_ROM = 110.0; 
        public static final double ARM_NEUTRAL_ANGLE = 90.0;
        public static final double ARM_MIN = 0.0;
        public static final double ARM_MAX = 120.0;
        public static final double ARM_SETPOINT_TOLERANCE = 3.0;
        public static final double ARM_NULL_RANGE = 300.0; // To 360 degrees
    }

    // Constants pertaining to electrical
    public static class Electrical {
        public static final String CANIVORE_BUS_NAME = "dabus"; 
    }
  
}

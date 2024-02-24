// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.util.OrbitPID;

import edu.wpi.first.math.util.Units;
import frc.lib.util.PIDConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean isReplay = true; // only used if sim

    public static class LED {
        public static final int LEDPort = 2;
        public static final float ENABLED_COLOR = 0.87f; // BLUE
        public static final float DISABLED_COLOR = 0.77f; // GREEN
        public static final float NOTE_COLOR = 0.6f; // ORANGE
        public static final float SOURCE_COLOR = 0.93f; // WHITE
        public static final float SCORING_COLOR = 0.61f; // RED
    }

    public static final int kDriverControllerPort = 0;

    public static class ClimbConstants {
        public static final int CLIMBER_LEAD_CAN_ID = 99;
        public static final int CLIMBER_SLAVE_CAN_ID = 98;
        public static final double CLIMBER_ENCODER_EXTENDED_HEIGHT_IN_ROTATIONS = 20.0;
        public static final double LEAD_CLIMBER_MOTOR__SPEED = 0.9;
        public static final double CLIMBER_ENCODER_RETRACTED_HEIGHT_IN_ROTATIONS = 0;
        public static final double CLIMBER_ENCODER_CLIMBED_HEIGHT_IN_ROTATIONS = 0;
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int LEDPort = 0;
    }

    public static class VisionConstants {
        public static final Transform3d robotToCam = // Made negative, z prev 0.7366
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(2),
                                Units.inchesToMeters(2),
                                Units.inchesToMeters(16)), // forward is positive X, left is positive Y, and up is
                                                           // positive Z.
                        new Rotation3d(
                                0,
                                Math.toRadians(-32.0),
                                0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
        public static final String cameraName = "Camera_Module_v1";
        public static final double maxNoiseError = 0.25; // meters
    }

    public static class ShintakeConstants {
        public static final double UNFEED_SPEED_BACK = 0.1;
        public static final double INTAKE_SPEED_BACK = 0.1;
        public static final double INITIAL_DEFAULT_INTAKE_SPEED = 0.1;
        public static final double SECOND_DEFAULT_INTAKE_SPEED = 0.2;
        public static final double DEFAULT_INTAKE_SPEED = 0.1;
        public static final double SHOOT_SPEED_FRONT = 0.5;
        public static final double SHOOT_VELOCITY_FRONT = 6250;
        public static final double TARGET_SHOOT_VELOCITY_SPEAKER = 6250;
        public static final double SHOOT_SPEED_BACK_AMP = 0.1;
        public static final double SHOOT_SPEED_BACK_SPEAKER = 1.0;
        public static final int RIGHT_SHOOTAKE_CAN_ID = 37;
        public static final int LEFT_SHOOTAKE_CAN_ID = 38;
        public static final int BACK_SHOOTAKE_ID = 35;
        public static final int SHINTAKE_SENSOR_PIN = 9;
        public static final double UNFEED_SPEED_FRONT = 0.05;
        public static final double OUTAKE_SPEED_BACK = 0.5;
        public static final double OUTAKE_SPEED_FRONT = 0.5;
    }

    public static class STPConstants { // random placeholder numbers
        public static final int ShintakePivot_MOTOR = 42;
        public static final int ShintakePivot_FOLLOW_MOTOR = 43;
        public static final double ShintakePivot_MAX_ANGLE = 2;
        public static final double ShintakePivot_MIN_ANGLE = 3;
        public static final int ShintakePivot_ENCODER_CHANNEL = 4;
        public static final double ShintakePivot_GEAR_RATIO = (40.0 / 11.0) * (20.0 / 1.0);
        public static final double ShintakePivot_ENCODER_OFFSET = 6;
    }
    
    // HOME_POSITION
    public static final double HOME_POSITION_STP = 0.0;
    public static final double HOME_POSITION_ACP = 30.0;

    // NOTE_SCORE_AMP_POSITION
    public static final double NOTE_SCORE_AMP_POSITION_STP = 140.0;
    public static final double NOTE_SCORE_AMP_POSITION_ACP = 80.0;

    // NOTE_SCORE_SUBWOOFER_SPEAKER_POSITION
    public static final double NOTE_SCORE_SPEAKER_POSITION_STP = 0.0;
    public static final double NOTE_SCORE_SPEAKER_POSITION_ACP = 60.0;

    // NOTE_SCORE_TRAP_POSITION
    public static final double NOTE_SCORE_TRAP_POSITION_STP = 140.0; // TODO: SET ACTUAL VALUES
    public static final double NOTE_SCORE_TRAP_POSITION_ACP = 80.0; // TODO: SET ACTUAL VALUES

    // NOTE_SCORE_DEFENDED_SPEAKER_POSITION
    public static final double NOTE_SCORE_DEFENDED_SPEAKER_POSITION_STP = 30.0;
    public static final double NOTE_SCORE_DEFENDED_SPEAKER_POSITION_ACP = 80.0;

    // NOTE_SCORE_PODIUM_SPEAKER_POSITION
    public static final double NOTE_SCORE_PODIUM_SPEAKER_POSITION_STP = 0;
    public static final double NOTE_SCORE_PODIUM_SPEAKER_POSITION_ACP = 45.0;

    // SOURCE_POSITION
    public static final double SOURCE_POSITION_STP = 180;
    public static final double SOURCE_POSITION_ACP = 50;

    // CLIMB_POSITION
    public static final double CLIMB_POSITION_STP = 180.0;
    public static final double CLIMB_POSITION_ACP = 80.0;

    public static class ACPConstants {
        // SHOULDER
        public static final int ACP_MOTOR_MASTER = 50;
        public static final int ACP_MOTOR_SLAVE = 51;
        public static final int ACP_ENCODER_CHANNEL = 0;

        public static final double ACP_ENCODER_OFFSET = 0.472;

        public static final double ACP_GEAR_RATIO = (11.0 / 52.0) * (30.0 / 68.0) * (1.0 / 20.0);
        public static final double ACP_MANUAL_OVERRIDE_RANGE = 10.0;
        public static final double MAX_ACP_ANGLE = 80.0;
        public static final double MIN_ACP_ANGLE = 0.0;   
    }

    /*
     * Swerve Constants (newly added ones)
     */
    public static final class Swerve {
        /* Module Specific Constants */
        public static final String[] MODULE_NAMES = {
                "Front Left",
                "Front Right",
                "Back Left",
                "Back Right"
        }; // module
           // #0,
           // #1, #2, #3

        public static int PEAK_CURRENT_LIMIT = 40;
        public static int CONTINUOUS_CURRENT_LIMIT = 40;
        public static boolean ANGLE_INVERT = true;
        public static boolean DRIVE_INVERT = true;
        public static boolean isGyroInverted = true;
        public static IdleMode IDLE_MODE = IdleMode.kBrake;

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = 0.61;
        public static final double WHEEL_BASE = 0.61;
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Drive Motor Conversion Factors */
        public static final double DRIVE_GEAR_RATIO = (6.75 / 1.0); // 6.75:1
        public static final double ANGLE_GEAR_RATIO = 150.0 / 7.0; // 150/7:1
        public static final double DRIVE_CONVERSION_POSITION_FACTOR = (WHEEL_DIAMETER * Math.PI) / DRIVE_GEAR_RATIO;
        public static final double DRIVE_CONVERSION_VELOCITY_FACTOR = DRIVE_CONVERSION_POSITION_FACTOR / 60.0;
        public static final double ANGLE_CONVERSION_FACTOR = 360.0 / ANGLE_GEAR_RATIO;
        // public static final double MAX_SPEED = 14.5 / 3.28084;
        public static final double MAX_SPEED = Swerve.AutoConstants.maxSpeed;

        /*
         * Ideally these should be independent but for getting started same pid/ff
         * values should work just fine
         */
        public static final PIDConstants drivePID = new PIDConstants(0.3, 0.0000, 0.0045);
        public static final SimpleMotorFeedforward driveSVA = new SimpleMotorFeedforward(0.1, 3, 0.4);
        public static final PIDConstants anglePID = new PIDConstants(0.023, 0.000001, 0.0);

        /* Custom PID Controllers */
        public static final OrbitPID robotRotationPID = new OrbitPID(0.1, 0, 0.00005);

        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;

            public static final double angleOffset = 360.0 - 218.93 + 180.0; // 130.0; // Always reversed direction, add
                                                                             // 180
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset, anglePID, drivePID, driveSVA);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 20;
            public static final int angleMotorID = 21;
            public static final int canCoderID = 22;

            public static final double angleOffset = 360.0 - 195.46; // 40.3;

            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset, anglePID, drivePID, driveSVA);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 30;
            public static final int angleMotorID = 31;
            public static final int canCoderID = 32;
            public static final double angleOffset = 360.0 - 158.73; // 252.2;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset, anglePID, drivePID, driveSVA);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 40;
            public static final int angleMotorID = 41;
            public static final int canCoderID = 42;
            public static final double angleOffset = 360.0 - 50.18; // 326.85;

            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset, anglePID, drivePID, driveSVA);
        }

        public static final class AutoConstants {
            // PID values to follow paths. NOT *DIRECTLY* FOR MODULE SPEED, try DRIVE_PID
            // and ANGLE_PID first
            public static final com.pathplanner.lib.util.PIDConstants translation = new com.pathplanner.lib.util.PIDConstants(
                    1, 0, 0.0045);
            public static final com.pathplanner.lib.util.PIDConstants rotation = new com.pathplanner.lib.util.PIDConstants(
                    0.05, 0.000001, 0);
            public static final double maxSpeed = 4; // m/s
            public static final double maxAcceleration = 16; // m/s^2
            public static final double maxAngularVelocity = Units.degreesToRadians(540); // d/s
            public static final double maxAngularAcceleration = Units.degreesToRadians(720); // deg/s^2
            public static final double positionTolerance = 0.025;
            public static final double angleTolerance = Math.toRadians(1);
        }

    }

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.61; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.61; // FIXME Measure and set wheelbase

    public static final double ROBOT_MAX_VELOCITY_METERS_PER_SECOND = 14.5 / 3.28084; // ft/s divide ft/m to convert to
    // m/s

    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 1 /
            Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    // SDS Module Configurations
    public static final double SWERVE_WHEEL_DIAMETER = 0.10033; // in meters
    public static final double SWERVE_DRIVE_GEAR_RATIO = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double SWERVE_STEER_GEAR_RATIO = (14.0 / 50.0) * (10.0 / 60.0);
    public static final double SWERVE_DRIVE_MOTOR_RAMP_RATE = 1.0; // Time is seconds for acceleration from 0 to full
    // speed
    public static final double SWERVE_STEER_MOTOR_RAMP_RATE = 1.0;
    public static final int SWERVE_ENCODER_PULSE_PER_REV = 1;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 10; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 11; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 0; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -(218.4075 + 90.0); // FIXME Measure and set front left
    // steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 20; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 21; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 1; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -(310.588 + 90.0); // FIXME Measure and set front right
    // steer offset
    // 2, 72.098
    // 1, 40.341

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 30; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 31; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 2; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -(342.0 + 90.0); // FIXME Measure and set back left steer
    // offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 40; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 41; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -(54.767 + 180.0 + 90.0); // FIXME Measure and set back
    // right steer offset

    public static final class Drivetrain {
        public static final double DRIVE_MOTION_PROFILE_MAX_VELOCITY = 4.000;
        public static final double DRIVE_MOTION_PROFILE_MAX_ACCELERATION = 3.550;
        public static final double ROTATION_MOTION_PROFILE_MAX_VELOCITY = 180.0;
        public static final double ROTATION_MOTION_PROFILE_MAX_ACCELERATION = 180.0;
    }

    // GENERAL
    public static double NEO_ENCODER_TICKS_PER_REV = 42;
}
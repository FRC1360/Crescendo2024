// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
  public static final int kDriverControllerPort = 0;
  
  // SHOULDER
  public static final int SHOULDER_MOTOR_MASTER = 50;
  public static final int SHOULDER_MOTOR_SLAVE = 51;
  public static final int SHOULDER_ENCODER = 0;

  // public static final double SHOULDER_ENCODER_OFFSET = 0.542;
  public static final double SHOULDER_ENCODER_OFFSET = 0.472;
  // 0.045 - 0.027 - 0.102777 - 0.02777 + 0.05 + 0.03888 + 0.02777
  //         - 0.02
  //         - 0.07222 + 0.041666; // Measure
  // // when
  // arm
  // is
  // parallel to
  // floor

  public static final double SHOULDER_GEAR_RATIO = (11.0 / 52.0) * (30.0 / 68.0) * (12.0 / 60.0);
  public static final double SHOULDER_MANUAL_OVERRIDE_RANGE = 10.0;
  public static final double MAX_SHOULDER_ANGLE = 90.0;
  public static final double MIN_SHOULDER_ANGLE = -150.0;
 // HOME_POSITION
 public static final double HOME_POSITION_WRIST = 170.0; // Originally 175.0
 public static final double HOME_POSITION_ARM = 0.0;
 public static final double HOME_POSITION_SHOULDER = -90.0; // -90.0, changed it for ribfest so that the neo doesn't
                                                            // hit the superstructure

 // CONE_INTAKE_POSITION
 public static final double CONE_INTAKE_POSITION_WRIST = 53.0;
 public static final double CONE_INTAKE_POSITION_ARM = 4.0;
 public static final double CONE_INTAKE_POSITION_SHOULDER = -48.0; // -48

 // CUBE_INTAKE_POSITION
 public static final double CUBE_INTAKE_POSITION_WRIST = 95.0;
 public static final double CUBE_INTAKE_POSITION_ARM = 5.5;
 public static final double CUBE_INTAKE_POSITION_SHOULDER = -52.0;

 // CONE_SCORE_HIGH_POSITION
 public static final double CONE_SCORE_HIGH_POSITION_WRIST = -33.5;
 public static final double CONE_SCORE_HIGH_POSITION_ARM = 12; // Originally 20
 public static final double CONE_SCORE_HIGH_POSITION_SHOULDER = 42.0;

 // CUBE_SCORE_HIGH_POSITION
 public static final double CUBE_SCORE_HIGH_POSITION_WRIST = 130.0;
 public static final double CUBE_SCORE_HIGH_POSITION_ARM = 5.0;
 public static final double CUBE_SCORE_HIGH_POSITION_SHOULDER = -7.5;

 // SCORE_MID_POSITION
 public static final double CONE_SCORE_MID_POSITION_WRIST = -35.0; // -35
 public static final double CONE_SCORE_MID_POSITION_ARM = HOME_POSITION_ARM;
 public static final double CONE_SCORE_MID_POSITION_SHOULDER = 31.0; // this was negative before for some reason lol

 // CUBE_SCORE_MID_POSITION
 public static final double CUBE_SCORE_MID_POSITION_WRIST = 170.0;
 public static final double CUBE_SCORE_MID_POSITION_ARM = 0.0;
 public static final double CUBE_SCORE_MID_POSITION_SHOULDER = -48.0;

 // SINGLE_SUBSTATION_POSITION
 public static final double SINGLE_SUBSTATION_POSITION_WRIST = 150.0;
 public static final double SINGLE_SUBSTATION_POSITION_ARM = HOME_POSITION_ARM;
 public static final double SINGLE_SUBSTATION_POSITION_SHOULDER = -48.0;

  }
}

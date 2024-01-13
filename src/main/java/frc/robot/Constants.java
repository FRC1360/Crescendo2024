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
  }

  public static class ACPConstants {
    // SHOULDER
    public static final int ACP_MOTOR_MASTER = 50;
    public static final int ACP_MOTOR_SLAVE = 51;
    public static final int ACP_ENCODER = 0;

    public static final double ACP_ENCODER_OFFSET = 0.472;

    public static final double ACP_GEAR_RATIO = (11.0 / 52.0) * (30.0 / 68.0) * (12.0 / 60.0);
    public static final double ACP_MANUAL_OVERRIDE_RANGE = 10.0;
    public static final double MAX_ACP_ANGLE = 90.0;
    public static final double MIN_ACP_ANGLE = -150.0;

  // HOME_POSITION
  public static final double HOME_POSITION_STP= 170.0; // Originally 175.0
  public static final double HOME_POSITION_ACP = -90.0; // -90.0, changed it for ribfest so that the neo doesn't
                                                              // hit the superstructure
  // NOTE_SCORE_AMP_POSITION
  public static final double NOTE_SCORE_AMP_POSITION_STP = -33.5;
  public static final double NOTE_SCORE_AMP_POSITION_ACP = 42.0;

  // NOTE_SCORE_SPEAKER_POSITION
  public static final double NOTE_SCORE_SPEAKER_POSITION_STP = 130.0;
  public static final double NOTE_SCORE_SPEAKER_POSITION_ACP = -7.5;

  // SOURCE_POSITION
  public static final double SOURCE_POSITION_STP = 150.0;
  public static final double SOURCE_POSITION_ACP = -48.0;
  }
}

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
  public static class STPConstants{ // random placeholder numbers
    public static final int ShintakePivot_MOTOR = 1;
    public static final int ShintakePivot_MAX_ANGLE = 2;
    public static final int ShintakePivot_MIN_ANGLE = 3;
    public static final int ShintakePivot_ENCODER = 4;
    public static final int ShintakePivot_GEAR_RATIO = 5;
    public static final int ShintakePivot_ENCODER_OFFSET = 6;

    public static final double HOME_POSITION_STP= 170.0; // Originally 175.0
    public static final double HOME_POSITION_ACP = -90.0; // -90.0, changed it for ribfest so that the neo doesn't
                                                                // hit the superstructure
    // NOTE_SCORE_AMP_POSITION
    public static final double NOTE_SCORE_AMP_POSITION_STP = -33.5;

    // NOTE_SCORE_SPEAKER_POSITION
    public static final double NOTE_SCORE_SPEAKER_POSITION_STP = 130.0;

    // SOURCE_POSITION
    public static final double SOURCE_POSITION_STP = 150.0;
  }
}

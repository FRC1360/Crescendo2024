package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AlignmentConstants {

    // A set of Pose2d which represents the targetPose where we want to align the
    // bot

    // TODO TUNE THESE VALUES

    // BLUE ALLIANCE!!!

    public static final Pose2d BLUE_SOURCE_LEFT = new Pose2d(15.89, 1.41, Rotation2d.fromDegrees(-57.69));
    public static final Pose2d BLUE_SOURCE_CENTER = new Pose2d(15.36, 1.04, Rotation2d.fromDegrees(-57.69)); // ID 1 & 2
    public static final Pose2d BLUE_SOURCE_RIGHT = new Pose2d(14.79, 0.73, Rotation2d.fromDegrees(-57.69));

    public static final Pose2d BLUE_AMP = new Pose2d(1.86, 7.60, Rotation2d.fromDegrees(-90.0)); // ID 6

    public static final Pose2d BLUE_SPEAKER = new Pose2d(1.34, 5.53, Rotation2d.fromDegrees(180.0)); // ID 7 & 8 ** 180 needs to be added NEED TO FACE AWAY FOR SHOT 

    // This is used to get into the speaker
    public static final Pose2d INTO_BLUE_SPEAKER = new Pose2d(0.2, 5.53, Rotation2d.fromDegrees(180.0)); 

    // STAGE with respect to blue alliance (far is cannot be seen)

    public static final Pose2d BLUE_STAGE_FAR = new Pose2d(6.64, 4.10, Rotation2d.fromDegrees(180.0)); // ID 14
    public static final Pose2d BLUE_STAGE_LEFT = new Pose2d(4.05, 5.50, Rotation2d.fromDegrees(-60.0)); // ID 15
    public static final Pose2d BLUE_STAGE_RIGHT = new Pose2d(4.02, 2.56, Rotation2d.fromDegrees(62.30)); // ID 16

    /// RED ALLIANCE!!!

    public static final Pose2d RED_SOURCE_CENTER = new Pose2d(1.20, 1.11, Rotation2d.fromDegrees(-119.51)); // ID 9 & 10
    public static final Pose2d RED_SOURCE_LEFT = new Pose2d(1.72, 0.75, Rotation2d.fromDegrees(-119.51));
    public static final Pose2d RED_SOURCE_RIGHT = new Pose2d(0.65, 1.38, Rotation2d.fromDegrees(-119.51));

    public static final Pose2d RED_SPEAKER = new Pose2d(15.12, 5.56, Rotation2d.fromDegrees(0.0)); // ID 3 & 4
    public static final Pose2d INTO_RED_SPEAKER = new Pose2d(16.26, 5.56, Rotation2d.fromDegrees(0.0)); 


    public static final Pose2d RED_AMP = new Pose2d(14.70, 7.60, Rotation2d.fromDegrees(-90.0)); // ID 5

    // STAGE with respect to red alliance (far is cannot be seen)
    public static final Pose2d RED_STAGE_FAR = new Pose2d(9.90, 4.10, Rotation2d.fromDegrees(0.0)); // ID 13
    public static final Pose2d RED_STAGE_LEFT = new Pose2d(12.63, 2.56, Rotation2d.fromDegrees(125.00)); // ID 11
    public static final Pose2d RED_STAGE_RIGHT = new Pose2d(12.67, 5.50, Rotation2d.fromDegrees(-125.00)); // ID 12

}

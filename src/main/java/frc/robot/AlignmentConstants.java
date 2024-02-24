package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AlignmentConstants {

    // A set of Pose2d which represents the targetPose where we want to align the
    // bot

    // TODO TUNE THESE VALUES

    // BLUE ALLIANCE!!!

    public static final Pose2d BLUE_SOURCE = new Pose2d(15.12, 1.81, Rotation2d.fromDegrees(-60.95)); // ID 1 & 2
    public static final Pose2d BLUE_AMP = new Pose2d(1.82, 7.30, Rotation2d.fromDegrees(90.0)); // ID 6

    public static final Pose2d BLUE_SPEAKER = new Pose2d(1.82, 5.53, Rotation2d.fromDegrees(0.0)); // ID 7 & 8 ** NEED TO FACE AWAY FOR SHOT 

    // STAGE with respect to blue alliance (far is cannot be seen)

    public static final Pose2d BLUE_STAGE_FAR = new Pose2d(6.64, 4.10, Rotation2d.fromDegrees(180.0)); // ID 14
    public static final Pose2d BLUE_STAGE_LEFT = new Pose2d(4.05, 5.50, Rotation2d.fromDegrees(-60.0)); // ID 15
    public static final Pose2d BLUE_STAGE_RIGHT = new Pose2d(4.02, 2.56, Rotation2d.fromDegrees(62.30)); // ID 16

    /// RED ALLIANCE!!!

    public static final Pose2d RED_SOURCE = new Pose2d(1.16, 1.49, Rotation2d.fromDegrees(-115.49)); // ID 9 & 10

    public static final Pose2d RED_SPEAKER = new Pose2d(14.92, 5.56, Rotation2d.fromDegrees(0.0)); // ID 3 & 4

    public static final Pose2d RED_AMP = new Pose2d(14.70, 7.30, Rotation2d.fromDegrees(90.0)); // ID 5

    // STAGE with respect to red alliance (far is cannot be seen)
    public static final Pose2d RED_STAGE_FAR = new Pose2d(9.90, 4.10, Rotation2d.fromDegrees(0.0)); // ID 13
    public static final Pose2d RED_STAGE_LEFT = new Pose2d(12.63, 2.56, Rotation2d.fromDegrees(125.00)); // ID 11
    public static final Pose2d RED_STAGE_RIGHT = new Pose2d(12.67, 5.50, Rotation2d.fromDegrees(-125.00)); // ID 12

}

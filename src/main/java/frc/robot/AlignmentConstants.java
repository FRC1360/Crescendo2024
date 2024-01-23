package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AlignmentConstants {
    
    // A set of Pose2d which represents the targetPose where we want to align the bot
    
    // TODO TUNE THESE VALUES

    public static final Pose2d RED_SOURCE = new Pose2d(2.2, 1.89, Rotation2d.fromDegrees(-129.0));
    
    public static final Pose2d BLUE_AMP = new Pose2d(1.82, 7.30, Rotation2d.fromDegrees(-90.0)); 

    public static final Pose2d BLUE_SPEAKER = new Pose2d(1.82, 5.53, Rotation2d.fromDegrees(-180.0)); 
}

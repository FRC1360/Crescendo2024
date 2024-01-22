package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AlignmentConstants {
    
    // A set of Pose2d which represents the targetPose where we want to align the bot
    
    // TODO TUNE THESE VALUES

    public static final Pose2d RED_SOURCE = new Pose2d(1.24, 1.70, Rotation2d.fromDegrees(120.0));
    
    public static final Pose2d BLUE_AMP = new Pose2d(1.82, 7.30, Rotation2d.fromDegrees(-90.0)); 
}

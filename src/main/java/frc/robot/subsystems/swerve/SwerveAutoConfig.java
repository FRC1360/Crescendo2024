package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public final class SwerveAutoConfig { 

    public static void configureAutoBuilder(SwerveSubsystem swerve) {  
        AutoBuilder.configureHolonomic(
            swerve::currentPose, 
                swerve::setCurrentPose,
                swerve::getRobotRelativeSpeeds, 
                swerve::driveRobotRelative, 
                new HolonomicPathFollowerConfig(
                    Constants.Swerve.AutoConstants.translation, 
                    Constants.Swerve.AutoConstants.rotation,
                    Constants.ROBOT_MAX_VELOCITY_METERS_PER_SECOND, 
                    Math.sqrt(Math.pow(Constants.Swerve.TRACK_WIDTH / 2, 2) + Math.pow(Constants.Swerve.WHEEL_BASE / 2, 2)), // Pythagorean distance for robot center to a module
                    new ReplanningConfig()
                ), 
                () -> false, 
            swerve
        );        
    }
}

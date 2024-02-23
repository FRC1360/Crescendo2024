package frc.robot.subsystems.swerve;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
            () -> { 
                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance(); 
                SmartDashboard.putBoolean("AutoBuilder isBlue: ", (alliance.get() != null && alliance.get() == DriverStation.Alliance.Blue)); 
                if (alliance.isPresent()) { 
                    return alliance.get() == DriverStation.Alliance.Red; 
                }
                return false; // Defaults to no path flipping if no alliance data available
            }, 
            swerve
        );
        System.out.println("AutoBuilder Configured!");
    }
}

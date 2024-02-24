package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignmentConstants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateForShot extends Command {
    
    private SwerveSubsystem swerveSubsystem; 
    public RotateForShot(SwerveSubsystem swerve) { 
        this.swerveSubsystem = swerve; 

        addRequirements(swerve);
    }

    public double convertDeltaYToAngle(double deltaY) { 
        return deltaY * 60; // 6.3-5.5 in y change would yield a 48 deg change from straight up
    }

    @Override 
    public void execute() { 
        // Rotate based on y coordinate
        // Center (y) = 5.5
        // Most right - 6.3

        Pose2d curPose = swerveSubsystem.currentPose(); 

        double targetAngle = convertDeltaYToAngle(curPose.getY() - AlignmentConstants.BLUE_SPEAKER.getY())
                                + AlignmentConstants.BLUE_SPEAKER.getRotation().getDegrees(); 

        if (DriverStation.getAlliance().isPresent() 
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) { 
            targetAngle = convertDeltaYToAngle(curPose.getY() - AlignmentConstants.RED_SPEAKER.getY()) 
                            + AlignmentConstants.RED_SPEAKER.getRotation().getDegrees(); ; 
        }

        double currentAngle = (curPose.getRotation().getRadians() + 2 * Math.PI) % (2 * Math.PI);

        if (Math.abs(targetAngle) < 20) {
            // If aligning near 0, use the -180 to 180 alignment (built in Rotation2d) to
            // prevent rollover
            currentAngle = curPose.getRotation().getRadians();
        }

        
    }
}

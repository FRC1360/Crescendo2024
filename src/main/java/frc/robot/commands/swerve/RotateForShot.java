package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignmentConstants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateForShot extends Command {
    
    private SwerveSubsystem swerveSubsystem; 
    private DoubleSupplier xDriveSupplier; 
    private DoubleSupplier yDriveSupplier; 

    public RotateForShot(SwerveSubsystem swerve, DoubleSupplier xDriveSupplier, DoubleSupplier yDriveSupplier) { 
        this.swerveSubsystem = swerve; 
        this.xDriveSupplier = xDriveSupplier; 
        this.yDriveSupplier = yDriveSupplier; 

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
            targetAngle = -convertDeltaYToAngle(curPose.getY() - AlignmentConstants.RED_SPEAKER.getY()) 
                            + AlignmentConstants.RED_SPEAKER.getRotation().getDegrees(); 
        }

        double rotPIDOut = this.swerveSubsystem.calculatePIDAngleOutput(targetAngle); 

        this.swerveSubsystem.drive(new Translation2d(xDriveSupplier.getAsDouble(), yDriveSupplier.getAsDouble()), rotPIDOut, true, false);
    }

    @Override
    public boolean isFinished() { 
        return false;
    }
}

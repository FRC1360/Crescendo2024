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

    public double convertToAngle() { 
        Pose2d robotPose = swerveSubsystem.currentPose(); 
        double deltaX = robotPose.getX() - AlignmentConstants.INTO_BLUE_SPEAKER.getX(); 
        double deltaY = robotPose.getY() - AlignmentConstants.INTO_BLUE_SPEAKER.getY(); 
        if (DriverStation.getAlliance().isPresent() 
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) { 
            deltaX = robotPose.getX() - AlignmentConstants.INTO_RED_SPEAKER.getX(); 
            deltaY = robotPose.getY() - AlignmentConstants.INTO_RED_SPEAKER.getY();
            return -Math.toDegrees(Math.atan(deltaY / deltaX)) + AlignmentConstants.RED_SPEAKER.getRotation().getDegrees(); //+ 180.0;;
        }
        return Math.toDegrees(Math.atan(deltaY / deltaX)) + AlignmentConstants.BLUE_SPEAKER.getRotation().getDegrees(); //- 180.0;
    }

    @Override 
    public void execute() { 

        double targetAngle = convertToAngle(); 

        double rotPIDOut = this.swerveSubsystem.calculatePIDAngleOutput(targetAngle); 

        this.swerveSubsystem.drive(new Translation2d(xDriveSupplier.getAsDouble(), yDriveSupplier.getAsDouble()), rotPIDOut, true, false);
    }

    @Override
    public boolean isFinished() { 
        return false;
    }
}

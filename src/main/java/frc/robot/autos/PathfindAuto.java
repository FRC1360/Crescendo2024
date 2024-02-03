package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.AutoConstants;
import frc.robot.commands.swerve.AlignToPose;
import frc.robot.subsystems.SwerveSubsystem;

public class PathfindAuto {

    private SwerveSubsystem swerveSubsystem;
    private Pose2d targetPose; 
    private PathConstraints constraints; 
    
    public PathfindAuto(SwerveSubsystem swerveSubsystem, Pose2d targetPose) { 
        this.targetPose = targetPose; 

        this.swerveSubsystem = swerveSubsystem; 

        this.constraints = new PathConstraints(Constants.Swerve.AutoConstants.maxSpeed, Constants.Swerve.AutoConstants.maxAcceleration,
                                                    Constants.Swerve.AutoConstants.maxAngularVelocity, 
                                                    Constants.Swerve.AutoConstants.maxAngularAcceleration); 

    }

    public Command getCommand() {
        return AutoBuilder.pathfindToPose(this.targetPose, constraints, 0.0, 0.5)
        .until(() -> swerveSubsystem.isInRange(targetPose, AutoConstants.positionTolerance * 20, AutoConstants.angleTolerance * 10))
        .alongWith(new InstantCommand(() -> System.out.println(this.targetPose)))
        .andThen(new AlignToPose(this.swerveSubsystem, targetPose));
    }
}

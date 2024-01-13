package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class PathfindAuto {

    private Pose2d targetPose; 
    private PathConstraints constraints; 
    
    public PathfindAuto(Pose2d targetPose) { 
        this.targetPose = targetPose; 

        this.constraints = new PathConstraints(Constants.Swerve.AutoConstants.maxSpeed, Constants.Swerve.AutoConstants.maxAcceleration,
                                                    Constants.Swerve.AutoConstants.maxAngularVelocity, 
                                                    Constants.Swerve.AutoConstants.maxAngularAcceleration); 

    }

    public Command getCommand() { 
        return AutoBuilder.pathfindToPose(this.targetPose, constraints, 0.0, 0.0); 
    }
}

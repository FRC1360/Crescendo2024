package frc.robot.autos;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
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
    private PathConstraints constraints;

    private Pose2d targetPose;

    private boolean allowEnd;

    public PathfindAuto(SwerveSubsystem swerveSubsystem, Pose2d targetPose) {
        this(swerveSubsystem, targetPose, false);
    }

    public PathfindAuto(SwerveSubsystem swerveSubsystem, Pose2d targetPose, boolean allowEnd) {
        this.targetPose = targetPose;

        
        this.swerveSubsystem = swerveSubsystem;

        this.constraints = new PathConstraints(Constants.Swerve.AutoConstants.maxSpeed,
                Constants.Swerve.AutoConstants.maxAcceleration,
                Constants.Swerve.AutoConstants.maxAngularVelocity,
                Constants.Swerve.AutoConstants.maxAngularAcceleration);

        this.allowEnd = allowEnd;
    }

    public Command getCommand() {
        if (!this.swerveSubsystem.manualDrive) {
            // return AutoBuilder.pathfindToPose(this.targetPose, constraints, 0.0, 0.5)
            // .until(() -> swerveSubsystem.isInRange(targetPose,
            // AutoConstants.positionTolerance,
            // AutoConstants.angleTolerance))
            // .alongWith(new InstantCommand(() -> System.out.println("Pathfinding to: " +
            // this.targetPose)))
            // .alongWith(new InstantCommand(() -> Logger.recordOutput("Swerve/TargetPose",
            // this.targetPose)))
            // .andThen(
            // return new InstantCommand(() ->
            // swerveSubsystem.setMotionProfileInit(false))//)
            // .alongWith(new InstantCommand(() -> Logger.recordOutput("Swerve/TargetPose",
            // this.targetPose)))
            // .andThen(new AlignToPose(this.swerveSubsystem, targetPose, allowEnd))
            // .andThen(new InstantCommand(() -> Logger.recordOutput("Swerve/TargetPose",
            // new Pose2d())));

            return new InstantCommand();
        }

        return new InstantCommand();
    }
}

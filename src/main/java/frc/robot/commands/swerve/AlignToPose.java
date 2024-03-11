package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.PIDSwerveValues;

public class AlignToPose extends Command {

    private SwerveSubsystem swerveSubsystem;
    private Pose2d target;

    private boolean allowEnd;

    public AlignToPose(SwerveSubsystem swerveSubsystem, Pose2d target) {
        this(swerveSubsystem, target, false);
    }

    public AlignToPose(SwerveSubsystem swerveSubsystem, Pose2d target, boolean allowEnd) {
        this.swerveSubsystem = swerveSubsystem;
        this.target = target;
        this.allowEnd = allowEnd;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        PIDSwerveValues pidOutput = this.swerveSubsystem.calculateControlLoopDriveOutput(this.target); 
        
        this.swerveSubsystem.drive(new Translation2d(pidOutput.xOut, pidOutput.yOut), pidOutput.rotationOut, true, false);
    }

    @Override
    public boolean isFinished() {
        return allowEnd && (this.swerveSubsystem.drivePIDAtTarget() || Math.abs(this.swerveSubsystem.calculateDistanceToTarget(this.target)) < 0.1); 
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new Translation2d(), 0, true, true);
    }
}

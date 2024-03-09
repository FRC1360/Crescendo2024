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

    public AlignToPose(SwerveSubsystem swerveSubsystem, Pose2d target) {
        this.swerveSubsystem = swerveSubsystem;
        this.target = target;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        PIDSwerveValues pidOutput = this.swerveSubsystem.calculatePIDDriveOutput(this.target); 
        
        this.swerveSubsystem.drive(new Translation2d(pidOutput.xOut, pidOutput.yOut), pidOutput.rotationOut, true, false);
    }

    @Override
    public boolean isFinished() {
        return this.swerveSubsystem.drivePIDAtTarget(); 
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new Translation2d(), 0, true, true);
    }
}

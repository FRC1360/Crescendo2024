package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToPose extends Command {

    private final double positionTolerance = 0.1;
    private final double angleTolerance = 1.0;
    
    private SwerveSubsystem swerveSubsystem;
    private Pose2d target;
    private PIDController drivePid = new PIDController(.5, 0, 0);
    private PIDController anglePid = new PIDController(1, 0, 0);
    private boolean done = false;

    public AlignToPose(SwerveSubsystem swerveSubsystem, Pose2d target) {
        this.swerveSubsystem = swerveSubsystem;
        this.target = target;
    }

    @Override
    public void initialize() {
        drivePid.reset();
        anglePid.reset();
        done = false;
    }

    @Override
    public void execute() {
        Transform2d error = swerveSubsystem.currentPose().minus(target);
        double driveOutput = drivePid.calculate(error.getTranslation().getNorm(), 0);
        double angleOutput = anglePid.calculate(error.getRotation().getDegrees());
        swerveSubsystem.drive(error.getTranslation().times(driveOutput), angleOutput, true, false);
        
        done = error.getRotation().getDegrees() < angleTolerance && error.getTranslation().getNorm() < positionTolerance;
    }

    @Override
    public boolean isFinished() {
      return done;
    }

    @Override
    public void end(boolean interrupted) {
        // stop
        swerveSubsystem.drive(new Translation2d(), 0, true, true);
    }
}

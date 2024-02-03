package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Swerve.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToPose extends Command {
    
    private SwerveSubsystem swerveSubsystem;
    private Pose2d target;
    private PIDController drivePid = new PIDController(5, 0, 0);
    private PIDController anglePid = new PIDController(5, 0, 0);
    private boolean done = false;   

    public AlignToPose(SwerveSubsystem swerveSubsystem, Pose2d target) {
        this.swerveSubsystem = swerveSubsystem;
        this.target = target;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        drivePid.reset();
        anglePid.reset();
        done = false;
    }

    @Override
    public void execute() {
        System.out.print("Aligning to pose: ");
        System.out.println(target);

        Transform2d error = target.minus(swerveSubsystem.currentPose());
        double driveOutput = drivePid.calculate(error.getTranslation().getNorm(), 0);
        double angleOutput = anglePid.calculate(error.getRotation().getRadians()); // Drive takes radians per second
        swerveSubsystem.drive(error.getTranslation().times(driveOutput), angleOutput, true, false);
        
        done = error.getRotation().getRadians() < AutoConstants.angleTolerance && error.getTranslation().getNorm() < AutoConstants.positionTolerance;
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

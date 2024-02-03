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
    private PIDController drivePid = new PIDController(0.8, 0, 0);
    private PIDController anglePid = new PIDController(1.5, 0, 0);
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

        System.out.print("Current: "); 

        Transform2d error = target.minus(swerveSubsystem.currentPose());
        double driveOutput = drivePid.calculate(error.getTranslation().getNorm(), 0.0);
        double angleOutput = anglePid.calculate(error.getRotation().getRadians(), 0.0); // Drive takes radians per second
        swerveSubsystem.drive(new Translation2d(error.getTranslation().getX()  * (driveOutput), 0
           // error.getTranslation().getY() * (driveOutput)
            ),
             angleOutput, true, false);
        //System.out.println("Drive" + error.getTranslation().times(driveOutput)); 
        //System.out.println("Angle" + angleOutput); 

        System.out.println("Current Pose: " + swerveSubsystem.currentPose()); 
        
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

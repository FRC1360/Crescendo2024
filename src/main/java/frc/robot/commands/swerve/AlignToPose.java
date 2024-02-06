package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private PIDController driveXPid = new PIDController(1.5, 0, 0);
    private PIDController driveYPid = new PIDController(1.5, 0, 0);
    private PIDController anglePid = new PIDController(1.5, 0, 0);  

    public AlignToPose(SwerveSubsystem swerveSubsystem, Pose2d target) {
        this.swerveSubsystem = swerveSubsystem;
        this.target = target;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        drivePid.reset();
        driveXPid.reset();
        driveYPid.reset();
        anglePid.reset();
    }

    @Override
    public void execute() {
        System.out.print("Aligning to pose: ");
        System.out.println(target);

        System.out.print("Current: "); 

        Pose2d currentPose = swerveSubsystem.currentPose(); 

        //Pose2d error = new Pose2d(new Translation2d(target.getX() - currentPose.getX(), target.getY() - currentPose.getY()), target.getRotation().minus(currentPose.getRotation())); 

       // Transform2d error = target.minus(swerveSubsystem.currentPose()); 
        //double driveOutput = drivePid.calculate(error.getTranslation().getNorm(), 0.0);
        //double angleOutput = anglePid.calculate(error.getRotation().getRadians(), 0.0); // Drive takes radians per second

        // Constrain angles between 0 and 360

        double currentAngle = (currentPose.getRotation().getRadians() + 2 * Math.PI) % (2 * Math.PI); 
        double targetAngle = (target.getRotation().getRadians() + 2 * Math.PI) % (2 * Math.PI); 

        System.out.println("Current angle: " + currentAngle + " Target Angle: " + targetAngle); 

        // Negated because a positive rotation is perceived as going right (as per joysticks)
        // But robot angle is positive rotating CCW
        double angleOutput = -anglePid.calculate(currentAngle, targetAngle); 
        // swerveSubsystem.drive(new Translation2d(error.getTranslation().getX()  * (driveOutput), 
        //     error.getTranslation().getY() * (driveOutput)
        //     ),
        //      angleOutput, true, false);
        //System.out.println("Drive" + error.getTranslation().times(driveOutput)); 
        //System.out.println("Angle" + angleOutput); 

        // double driveXOut = driveXPid.calculate(error.getX(), 0.0); 
        // double driveYOut = driveYPid.calculate(error.getY(), 0.0);

        double driveXOut = driveXPid.calculate(currentPose.getX(), target.getX());
        double driveYOut = driveXPid.calculate(currentPose.getY(), target.getY()); 

        swerveSubsystem.drive(new Translation2d(driveXOut, driveYOut), angleOutput, true, false); 

        System.out.println("Current Pose: " + swerveSubsystem.currentPose());

        System.out.println("Error X: " + driveXPid.getPositionError() + " Error Y: " + driveYPid.getPositionError()); 
        System.out.println("Error Angle" + anglePid.getPositionError()); 
         
        
        // done = error.getRotation().getRadians() < AutoConstants.angleTolerance && error.getTranslation().getNorm() < AutoConstants.positionTolerance;

        //done = error.getRotation().getRadians() < AutoConstants.angleTolerance && error.getX() < AutoConstants.positionTolerance && error.getY() < AutoConstants.positionTolerance;
    }

    @Override
    public boolean isFinished() {
        return driveXPid.atSetpoint() && driveYPid.atSetpoint() && anglePid.atSetpoint(); 
      //return done;
    }

    @Override
    public void end(boolean interrupted) {
        // stop
        swerveSubsystem.drive(new Translation2d(), 0, true, true);
    }
}

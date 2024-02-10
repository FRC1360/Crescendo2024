package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToPose extends Command {
    
    private SwerveSubsystem swerveSubsystem;
    private Pose2d target;
    private PIDController driveXPid = new PIDController(1.25, 0, 0);
    private PIDController driveYPid = new PIDController(1.25, 0, 0);
    private PIDController anglePid = new PIDController(1.25, 0, 0);  

    public AlignToPose(SwerveSubsystem swerveSubsystem, Pose2d target) {
        this.swerveSubsystem = swerveSubsystem;
        this.target = target;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
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

        // Constrain angles between 0 and 360

        double currentAngle = (currentPose.getRotation().getRadians() + 2 * Math.PI) % (2 * Math.PI); 
        double targetAngle = (target.getRotation().getRadians() + 2 * Math.PI) % (2 * Math.PI); 

        System.out.println("Current angle: " + currentAngle + " Target Angle: " + targetAngle); 

        // Negated because a positive rotation is perceived as going CW (for the joysticks)
        // But robot angle is positive rotating CCW
        double angleOutput = -anglePid.calculate(currentAngle, targetAngle); 

        double driveXOut = driveXPid.calculate(currentPose.getX(), target.getX());
        double driveYOut = driveXPid.calculate(currentPose.getY(), target.getY()); 

        swerveSubsystem.drive(new Translation2d(driveXOut, driveYOut), angleOutput, true, false); 

        System.out.println("Current Pose: " + swerveSubsystem.currentPose());

        System.out.println("Error X: " + driveXPid.getPositionError() + " Error Y: " + driveYPid.getPositionError()); 
        System.out.println("Error Angle" + anglePid.getPositionError()); 
    }

    @Override
    public boolean isFinished() {
        return driveXPid.atSetpoint() && driveYPid.atSetpoint() && anglePid.atSetpoint(); 
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new Translation2d(), 0, true, true);
    }
}

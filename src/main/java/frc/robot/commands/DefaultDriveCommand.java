package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends Command {
    private final SwerveSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final CommandJoystick rotationJoystick;

    public DefaultDriveCommand(SwerveSubsystem drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            CommandJoystick rotationJoystick) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.rotationJoystick = rotationJoystick;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement

        double rotSpeed = -m_rotationSupplier.getAsDouble();

        double curAngle = m_drivetrainSubsystem.navX.getYaw().getDegrees() % 360.0;

        double curAngle2 = (m_drivetrainSubsystem.navX.getYaw().getDegrees() + 180.0) % 360.0;

        SmartDashboard.putNumber("Default_Drive_Command_Cur_Angle", curAngle);
        // 2 -> 0 (180deg in alternative frame of reference)
        // 5 -> 90
        // 3 -> 180
        // 4 -> 270

        if (this.rotationJoystick.button(3).getAsBoolean())
            rotSpeed = -Constants.Swerve.robotRotationPID.calculate(180.0, curAngle);
        else if (this.rotationJoystick.button(2).getAsBoolean())
            rotSpeed = -Constants.Swerve.robotRotationPID.calculate(180.0, curAngle2);
        else if (this.rotationJoystick.button(5).getAsBoolean())
            rotSpeed = -Constants.Swerve.robotRotationPID.calculate(90.0, curAngle);
        else if (this.rotationJoystick.button(4).getAsBoolean()) {
            rotSpeed = -Constants.Swerve.robotRotationPID.calculate(270.0,
                    curAngle + (Math.abs(270 - curAngle) > 180 ? 360 : 0));
        }

        m_drivetrainSubsystem.drive(
                new Translation2d(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble()),
                rotSpeed,
                true, false);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new Translation2d(), 0, true, true);
    }

    private double getDirection(double target, double current) {
        if (target == current) {
            return 0;
        }
        if (Math.abs(current - target) < 180.0) {
            if (current < target) {
                return 1.0;
            } else {
                return -1.0;
            }
        } else {
            if (current < target) {
                return -1.0;
            } else {
                return 1.0;
            }
        }
    }
}

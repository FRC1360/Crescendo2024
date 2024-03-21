// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shintake;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShintakeSubsystem;

public class DefaultIntakeCommand extends Command {

    private ShintakeSubsystem intake;

    public DefaultIntakeCommand(ShintakeSubsystem intake) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.stopShooter();
        intake.stopIntake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.varIntake(0.1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
        // intake.resetShintakeCount();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
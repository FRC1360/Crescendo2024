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

public class IntakeCommand extends Command {

	private ShintakeSubsystem intake;
	private LEDSubsystem ledSubsystem;

	public IntakeCommand(ShintakeSubsystem intake, LEDSubsystem ledSubsystem) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.ledSubsystem = ledSubsystem;
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
		intake.varIntake(Constants.ShintakeConstants.INITIAL_DEFAULT_INTAKE_SPEED);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		intake.stopIntake();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
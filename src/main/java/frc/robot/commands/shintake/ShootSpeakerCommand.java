// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shintake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ShintakeSubsystem;
import frc.robot.util.OrbitTimer;

public class ShootSpeakerCommand extends Command {

	private ShintakeSubsystem m_shooter;
	// private int count_detect;
	private OrbitTimer timer = new OrbitTimer();
	private boolean ready2;

	public ShootSpeakerCommand(ShintakeSubsystem shooter) {
		// Use addRequirements() here to declare subsystem dependencies.

		this.m_shooter = shooter;
		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		// m_shooter.stopIntake();
		m_shooter.resetShintakeCount();
		// count_detect = 0;
		ready2 = false;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		SmartDashboard.putNumber("Left shooter velocity", m_shooter.getVelocityLeft());
		SmartDashboard.putNumber("Right shooter velocity", m_shooter.getVelocityRight());

		if (m_shooter.shooterWheelsReady() && !ready2) {
			ready2 = true;
			timer.start();
		}
		if (timer.getTimeDeltaMillis() >= 250 && ready2)
			m_shooter.varIntake(Constants.ShintakeConstants.SHOOT_SPEED_BACK_SPEAKER);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_shooter.stopIntake();
		m_shooter.stopShooter();
		ready2 = false;
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shintake;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ShintakeSubsystem;

public class ShootSpeakerFullCommand extends SequentialCommandGroup {

	private ShintakeSubsystem shooter;
	private CommandXboxController xboxController;

	/** Creates a new ShootSpeakerFullCommand. */
	public ShootSpeakerFullCommand(ShintakeSubsystem shooter) {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				new InstantCommand(() -> shooter.setVelocity(Constants.ShintakeConstants.TARGET_SHOOT_VELOCITY_SPEAKER,
						Constants.ShintakeConstants.TARGET_SHOOT_VELOCITY_SPEAKER))
						.alongWith(new OutakeCommand(shooter) /* REPLACE OUTAKE COMMAND WITH GO TO POSITION COMMAND */)
						.onlyIf(() -> shooter.getShooterReady(shooter.getBackEncoder()))
						.andThen(new ShootSpeakerCommand(shooter)).onlyIf(xboxController.b()));
	}

}

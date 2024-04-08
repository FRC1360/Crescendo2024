// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shintake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.ArmChassisPivot.ACPGoToPositionCommand;
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.subsystems.ShintakeSubsystem;

public class ShootSpeakerFullCommand extends SequentialCommandGroup {

	/** Creates a new ShootSpeakerFullCommand. */
	public ShootSpeakerFullCommand(ShintakeSubsystem shooter, ArmChassisPivotSubsystem ACP) {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(
				new InstantCommand(() -> shooter.clearFaults()),
				new MoveBackNoteCommand(shooter),
				new InstantCommand(() -> shooter.setVelocity(Constants.ShintakeConstants.TARGET_SHOOT_VELOCITY_SPEAKER,
						Constants.ShintakeConstants.TARGET_SHOOT_VELOCITY_SPEAKER))
		// .alongWith(new ACPGoToPositionCommand(ACP,
		// Constants.NOTE_SCORE_SPEAKER_POSITION_ACP)/* REPLACE OUTAKE COMMAND WITH GO
		// TO POSITION COMMAND */)
		);
	}

}

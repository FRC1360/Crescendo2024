// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shintake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ShintakeSubsystem;



public class ShootSpeakerFullCommand extends SequentialCommandGroup {

  private ShintakeSubsystem shooter;
  /** Creates a new ShootSpeakerFullCommand. */
  public ShootSpeakerFullCommand(ShintakeSubsystem shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ShootSpeakerCommand(shooter), 
    new InstantCommand(() -> shooter.setVelocity(Constants.ShintakeConstants.SHOOT_VELOCITY_FRONT, Constants.ShintakeConstants.SHOOT_VELOCITY_FRONT)),
    );
  }


}

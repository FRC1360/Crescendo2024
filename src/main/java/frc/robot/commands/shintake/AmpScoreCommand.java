// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShintakeSubsystem;
import frc.robot.util.StateMachine;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpScoreCommand extends Command {

  private ShintakeSubsystem shintakeSubsystem; 

  /** Creates a new AmpScoreCommand. */
  public AmpScoreCommand(ShintakeSubsystem shintakeSubsystem, LEDSubsystem ledSubsystem, StateMachine sm) {
    this.shintakeSubsystem = shintakeSubsystem; 
  }

  @Override
  public void initialize() { 
      shintakeSubsystem.setVelocity(Constants.ShintakeConstants.AMP_VELOCITY_FRONT, Constants.ShintakeConstants.AMP_VELOCITY_FRONT);
  }

  @Override
  public void execute() { 
      shintakeSubsystem.varIntake(Constants.ShintakeConstants.SHOOT_SPEED_BACK_AMP);
  }

  @Override
  public void end(boolean interrupted) { 
    this.shintakeSubsystem.stopShooter();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShintakeSubsystem;

public class ShootSpeakerCommand extends Command {

  private ShintakeSubsystem m_shooter;
 
  public ShootSpeakerCommand(ShintakeSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_shooter.stopIntake();
    m_shooter.stopShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.varShoot(Constants.ShintakeConstants.SHOOT_SPEED_FRONT);
    m_shooter.varIntake(Constants.ShintakeConstants.SHOOT_SPEED_BACK_SPEAKER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopIntake();
    m_shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

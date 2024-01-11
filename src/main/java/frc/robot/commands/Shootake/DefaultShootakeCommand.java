// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shootake;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShootakeSubsystem;

public class DefaultShootakeCommand extends Command {

  private ShootakeSubsystem m_shooter;

  public DefaultShootakeCommand(ShootakeSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_shooter = shooter;

    addRequirements(shooter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.varIntake(40);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

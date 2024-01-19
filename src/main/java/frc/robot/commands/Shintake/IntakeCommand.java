// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShintakeSubsystem;

public class IntakeCommand extends Command {

  private ShintakeSubsystem m_intake;

  public IntakeCommand(ShintakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_intake = intake;
    addRequirements(intake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.intakePiece();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.getDigitalInput();
  }
}

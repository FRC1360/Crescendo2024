// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShintakeSubsystem;

public class IntakeCommand extends Command {

  private ShintakeSubsystem m_intake;
  private boolean m_detect = false;

  public IntakeCommand(ShintakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    m_intake.stopShooter();
    m_intake.stopIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.varIntake(Constants.ShintakeConstants.INITIAL_DEFAULT_INTAKE_SPEED);
    if (!m_intake.getDigitalInput()) {
      m_intake.varIntake(-Constants.ShintakeConstants.UNFEED_SPEED_BACK);
      m_detect = true;
    }
    if (m_detect == true && m_intake.getDigitalInput()) m_intake.stopIntake();  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntake();
    m_detect = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

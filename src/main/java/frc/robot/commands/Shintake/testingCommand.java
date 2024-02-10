// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shintake;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ShintakeSubsystem;

public class testingCommand extends Command {

  private ShintakeSubsystem intake;
  private CommandXboxController xbox;
  private int count = 0;

  public testingCommand(ShintakeSubsystem intake, CommandXboxController xbox) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.xbox = xbox;
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.resetShintakeCount();
    intake.stopShooter();
    intake.stopIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xbox.x().whileTrue(new FeedCommand(intake));
    xbox.a().whileTrue(new ShootSpeakerCommand(intake));
    //count = m_intake.getShintakeCount();
    //if (!m_intake.getDigitalInput()) m_intake.varIntake(-Constants.ShintakeConstants.UNFEED_SPEED_BACK);
    //if (!m_intake.getDigitalInput()) m_intake.stopIntake(); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    intake.stopShooter();
    count = 0;
    intake.resetShintakeCount();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shintake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShintakeSubsystem;

public class ShootSpeakerCommand extends Command {

  private ShintakeSubsystem m_shooter;
  private boolean ready;
  private int count_undetect;
  private boolean prev;
 
  public ShootSpeakerCommand(ShintakeSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_shooter.stopIntake();
    m_shooter.stopShooter();
    m_shooter.resetShintakeCount();
    ready = false;
    count_undetect = 0; 
    prev = false; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_shooter.varShoot(Constants.ShintakeConstants.SHOOT_SPEED_FRONT);
    //if ((m_shooter.getVelocityLeft() >= 0.49 && m_shooter.getVelocityLeft() <= 0.51) && (m_shooter.getVelocityRight() >= 0.49 && m_shooter.getVelocityRight() <= 0.51)) m_shooter.varIntake(Constants.ShintakeConstants.SHOOT_SPEED_BACK_SPEAKER);

    m_shooter.varIntake(-Constants.ShintakeConstants.UNFEED_SPEED_BACK);
    if (m_shooter.getDigitalInput() && prev) { 
      count_undetect++; 
    }
    prev = m_shooter.getDigitalInput(); 
    if (!m_shooter.getDigitalInput()) count_undetect = 0; // A count to reduce noise
    
    if (count_undetect >= 3) ready = true;
    SmartDashboard.putNumber("Shooter Digital", m_shooter.getDigitalInput() ? 1 : 0); 
    if (ready) {
      m_shooter.stopIntake();
      m_shooter.setVelocity(Constants.ShintakeConstants.SHOOT_VELOCITY_FRONT,Constants.ShintakeConstants.SHOOT_VELOCITY_FRONT);
      if (m_shooter.shooterWheelsReady()) m_shooter.varIntake(Constants.ShintakeConstants.SHOOT_SPEED_BACK_SPEAKER);
      }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopIntake();
    m_shooter.stopShooter();
    ready = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

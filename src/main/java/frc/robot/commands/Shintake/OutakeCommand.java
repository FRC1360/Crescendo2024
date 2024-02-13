package frc.robot.commands.shintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShintakeSubsystem;

public class OutakeCommand extends Command {

  private ShintakeSubsystem m_intake;
  //private int m_count;

  public OutakeCommand(ShintakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    //m_count = 0;
    m_intake.stopShooter();
    m_intake.stopIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.varShoot(-Constants.ShintakeConstants.UNFEED_SPEED_BACK);
    m_intake.varIntake(-Constants.ShintakeConstants.UNFEED_SPEED_FRONT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntake();
    m_intake.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


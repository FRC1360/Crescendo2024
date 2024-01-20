package frc.robot.commands.Shintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShintakeSubsystem;

public class ShootTrapCommand extends Command {

  private ShintakeSubsystem m_shooter;
 
  public ShootTrapCommand(ShintakeSubsystem shooter) {
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
    m_shooter.varShootAmp(Constants.ShintakeConstants.SHOOT_SPEED_FRONT);
    m_shooter.varIntakeAmp(Constants.ShintakeConstants.SHOOT_SPEED_BACK_AMP);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
    m_shooter.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

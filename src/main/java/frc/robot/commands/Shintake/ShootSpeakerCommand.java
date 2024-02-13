// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shintake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShintakeSubsystem;

public class ShootSpeakerCommand extends Command {

  private ShintakeSubsystem shooter;
  private double leftVelo;
  private double rightVelo;
 
  public ShootSpeakerCommand(ShintakeSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.stopIntake();
    shooter.stopShooter();

    SmartDashboard.putNumber("left encoder value", shooter.getVelocityLeft());
    SmartDashboard.putNumber("right encoder value", shooter.getVelocityRight());
    SmartDashboard.putNumber("target encoder value", 5676);
    leftVelo = 5676;
    rightVelo = 5676;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_shooter.varShoot(Constants.ShintakeConstants.SHOOT_SPEED_FRONT);
    //if ((m_shooter.getVelocityLeft() >= 0.49 && m_shooter.getVelocityLeft() <= 0.51) && (m_shooter.getVelocityRight() >= 0.49 && m_shooter.getVelocityRight() <= 0.51)) m_shooter.varIntake(Constants.ShintakeConstants.SHOOT_SPEED_BACK_SPEAKER);

    shooter.setVelocity(Constants.ShintakeConstants.TARGET_SHOOT_VELOCITY_RIGHT_BACK_SPEAKER,Constants.ShintakeConstants.TARGET_SHOOT_VELOCITY_LEFT_BACK_SPEAKER);

    //if (shooter.shooterWheelsReady()) shooter.varIntake(Constants.ShintakeConstants.SHOOT_SPEED_BACK_SPEAKER);
    leftVelo = SmartDashboard.getNumber("Left Velocity", leftVelo);
    rightVelo = SmartDashboard.getNumber("Right Velocity", rightVelo);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopIntake();
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

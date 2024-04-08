// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShintakeSubsystem;

public class ShootTrapCommand extends Command {

  private ShintakeSubsystem shooter;

  public ShootTrapCommand(ShintakeSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.stopIntake();
    shooter.stopShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.varShoot(Constants.ShintakeConstants.SHOOT_SPEED_FRONT_TRAP);
    shooter.varIntake(Constants.ShintakeConstants.SHOOT_SPEED_BACK_AMP);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    shooter.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

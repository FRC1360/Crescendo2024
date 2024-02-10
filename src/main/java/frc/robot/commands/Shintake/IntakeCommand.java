// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shintake;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShintakeSubsystem;

public class IntakeCommand extends Command {

  private ShintakeSubsystem intake;
  private int count = 0;

  public IntakeCommand(ShintakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.

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
    intake.varIntake(Constants.ShintakeConstants.INITIAL_DEFAULT_INTAKE_SPEED);
    //count = intake.getShintakeCount();
    if (!intake.getDigitalInput()) intake.varIntake(-Constants.ShintakeConstants.UNFEED_SPEED_BACK);
    if (intake.getDigitalInput()) intake.stopIntake(); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    count = 0;
    intake.resetShintakeCount();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

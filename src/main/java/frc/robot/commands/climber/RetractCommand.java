// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.lang.constant.Constable;
import java.lang.invoke.ConstantBootstraps;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class RetractCommand extends Command {

    private ClimberSubsystem climb;

    public RetractCommand(ClimberSubsystem climb) {
        this.climb = climb;
        addRequirements(climb);
  }

  // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      climb.stopClimber();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if (climb.getIsExtended())
        climb.goToPosition(Constants.ClimbConstants.CLIMBER_ENCODER_RETRACTED_HEIGHT_IN_ROTATIONS, Constants.ClimbConstants.LEAD_CLIMBER_MOTOR__SPEED);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      climb.stopClimber();
      climb.setIsExtended(false);
      climb.setIsSafe(true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}

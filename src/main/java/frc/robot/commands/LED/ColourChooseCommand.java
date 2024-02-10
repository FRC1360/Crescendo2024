// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import com.ctre.phoenix6.signals.Led1OffColorValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class ColourChooseCommand extends Command {

  private int colourInput;
  private LEDSubsystem ledSubsystem;
  public ColourChooseCommand(LEDSubsystem ledSubsystem, int colourInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ledSubsystem = ledSubsystem;
    this.colourInput = colourInput;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (this.colourInput){
      case 1:
        this.ledSubsystem.setLEDDEFAULT();
        break;
      case 2: 
        this.ledSubsystem.setLEDAMP();
        break;
      case 3:
        this.ledSubsystem.setLEDPODIUM_SPEAKER();
        break;
      case 4:
        this.ledSubsystem.setLEDNOTE();
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

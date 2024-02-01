// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.OrbitPID;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class GoToHomeCommand extends Command {

  private final ClimberSubsystem m_climber;
  private final CommandXboxController m_cont;
  private OrbitPID heightPID;
  private double last_err = Double.NaN;

  public GoToHomeCommand(ClimberSubsystem climber, CommandXboxController cont) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_climber = climber;
    this.m_cont = cont;
    this.heightPID = new OrbitPID(0, 0, 0);
    /*SmartDashboard.putNumber("ki", 0);
    SmartDashboard.putNumber("kp", 0);
    SmartDashboard.putNumber("kd", 0);*/
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    m_climber.setEncoderPosition(Constants.ClimbConstants.CLIMBER_ENCODER_EXTENDED_HEIGHT_IN_ROTATIONS);;
    double kI = SmartDashboard.getNumber("ki", 0);
    double kP = SmartDashboard.getNumber("kp", 0);
    double kD = SmartDashboard.getNumber("kd", 0);
    heightPID.configure(kP, kI, kD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // use position to get height and use that instead of value
    last_err = m_climber.setPosition(Constants.ClimbConstants.CLIMBER_ENCODER_EXTENDED_HEIGHT_IN_ROTATIONS);

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return Math.abs(last_err) < 0.1 && Math.abs(last_err) > -0.1;
  }
}

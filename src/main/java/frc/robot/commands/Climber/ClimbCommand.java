// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.OrbitPID;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbCommand extends Command {

  private final ClimberSubsystem m_climber;
  private OrbitPID heightPID;
  private double last_err = Double.NaN;

  public ClimbCommand(ClimberSubsystem climber) {
    this.m_climber = climber;
    this.heightPID = new OrbitPID(0, 0, 0);
    /*SmartDashboard.putNumber("ki", 0);
    SmartDashboard.putNumber("kp", 0);
    SmartDashboard.putNumber("kd", 0);*/
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    m_climber.setEncoderPosition(Constants.ClimbConstants.CLIMBER_ENCODER_EXTENDED_HEIGHT_IN_ROTATIONS);
    double kI = SmartDashboard.getNumber("ki", 0);
    double kP = SmartDashboard.getNumber("kp", 0);
    double kD = SmartDashboard.getNumber("kd", 0);
    heightPID.configure(kP, kI, kD);
  }

  @Override
  public void execute() {
    last_err = m_climber.setPosition(Constants.ClimbConstants.CLIMBER_ENCODER_EXTENDED_HEIGHT_IN_ROTATIONS);

  }

  @Override
  public void end(boolean interrupted) {
    m_climber.stopClimber();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(last_err) < 0.1 && Math.abs(last_err) > -0.1;
  }
}

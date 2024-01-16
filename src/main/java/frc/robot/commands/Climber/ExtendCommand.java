// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.OrbitPID;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

public class ExtendCommand extends Command {

  private final ClimberSubsystem m_climber;
  private final CommandXboxController m_cont;
  private OrbitPID heightPID;
  private double last_err = Double.NaN;
  private RelativeEncoder m_extendEncoderLead;
  private CANSparkMax m_climbMotor;

  public ExtendCommand(ClimberSubsystem climber, CommandXboxController cont) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_climber = climber;
    this.m_cont = cont;
    this.m_climbMotor = m_climber.getClimbMotor();
    this.m_extendEncoderLead = m_climber.getRelativeClimbEncoder();
    this.heightPID = new OrbitPID(0, 0, 0);
    /*SmartDashboard.putNumber("ki", 0);
    SmartDashboard.putNumber("kp", 0);
    SmartDashboard.putNumber("kd", 0);*/
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    m_extendEncoderLead.setPosition(0);
    double kI = SmartDashboard.getNumber("ki", 0);
    double kP = SmartDashboard.getNumber("kp", 0);
    double kD = SmartDashboard.getNumber("kd", 0);
    heightPID.configure(kP, kI, kD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("lead encoder value", m_extendEncoderLead.getPosition());
    SmartDashboard.putNumber("target height", Constants.ClimbConstants.Climber_ENCODER_HEIGHT_ROTATIONS);

    // We need the actual height in motor roations
    double output = heightPID.calculate(Constants.ClimbConstants.Climber_ENCODER_HEIGHT_ROTATIONS, m_extendEncoderLead.getPosition());
    m_climbMotor.set(output);

    last_err = Constants.ClimbConstants.Climber_ENCODER_HEIGHT_ROTATIONS - m_extendEncoderLead.getPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbMotor.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return Math.abs(last_err) < 0.1 && Math.abs(last_err) > -0.1;
  }
}

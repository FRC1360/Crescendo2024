// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.OrbitPID;

public class ClimberSubsystem extends SubsystemBase {

private final CANSparkMax m_climbMotorLead;
private final CANSparkMax m_climbMotorSlave;
private OrbitPID highPID;
private double targetHeight;
private double currentHeight;
private final RelativeEncoder m_climbEncoderLead;
private final RelativeEncoder m_climbEncoderSlave;

public ClimberSubsystem() {
  this.m_climbMotorLead = new CANSparkMax(Constants.ClimbConstants.CLIMBER_LEAD_CAN_ID, MotorType.kBrushless);
  this.m_climbMotorSlave = new CANSparkMax(Constants.ClimbConstants.CLIMBER_SLAVE_CAN_ID, MotorType.kBrushless);;
  this.m_climbEncoderLead = m_climbMotorLead.getEncoder();
  this.m_climbEncoderSlave = m_climbMotorSlave.getEncoder();
  this.highPID.configure(0, 0, 0);
}

public RelativeEncoder getRelativeClimbEncoder() {
  return m_climbMotorLead.getEncoder();
}

public CANSparkMax getClimbMotor() {
  return m_climbMotorLead;
}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current height", currentHeight);
    SmartDashboard.putNumber("Target height", targetHeight);
  }
}

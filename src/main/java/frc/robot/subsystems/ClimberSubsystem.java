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
private Boolean isExtended = false;

public ClimberSubsystem() {
  this.m_climbMotorLead = new CANSparkMax(Constants.ClimbConstants.CLIMBER_LEAD_CAN_ID, MotorType.kBrushless);
  this.m_climbMotorSlave = new CANSparkMax(Constants.ClimbConstants.CLIMBER_SLAVE_CAN_ID, MotorType.kBrushless);;
  this.highPID.configure(0, 0, 0);

  m_climbMotorSlave.follow(m_climbMotorLead);
}

public RelativeEncoder getRelativeClimbEncoder() {
  return m_climbMotorLead.getEncoder();
}

public CANSparkMax getClimbMotor() {
  return m_climbMotorLead;
}

public CANSparkMax getClimbMotorInverted() {
  final CANSparkMax m_climbMotorInverted = m_climbMotorLead;
  m_climbMotorInverted.setInverted(true);
  return m_climbMotorInverted;
}

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("is extended: ", isExtended);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ShootakeSubsystem extends SubsystemBase {

  private CANSparkMax m_left;
  private CANSparkMax m_right;

  public ShootakeSubsystem() {
    this.m_left = new CANSparkMax(Constants.OperatorConstants.LEFT_SHOOTAKE_CAN_ID, MotorType.kBrushless);
    this.m_right = new CANSparkMax(Constants.OperatorConstants.RIGHT_SHOOTAKE_CAN_ID, MotorType.kBrushless);
  }
  
  //Moves both motors in the direction to intake when given a positive number
  //When not inverted, motors turn right
  public void intakePiece() {
    m_left.set(Constants.OperatorConstants.INTAKE_SPEED);
    m_right.setInverted(true);
    m_right.set(Constants.OperatorConstants.INTAKE_SPEED);
  }

  //Moves both motors in the direction to shoot when given a positive number
  public void shootPiece() {
    m_left.setInverted(true);
    m_left.set(Constants.OperatorConstants.SHOOT_SPEED);
    m_right.set(Constants.OperatorConstants.SHOOT_SPEED);
  }

  //Stops both motors
  public void stop() {
    m_left.set(0.0);
    m_right.set(0.0);
  }

  public void varIntake(int speed) {
    m_left.set(speed);
    m_right.setInverted(true);
    m_right.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (m_left.get() == 0 && m_right.get() == 0) {
      SmartDashboard.putBoolean("Intake in motion: ", false);
    }
    else {
      SmartDashboard.putBoolean("Intake in motion: ", true);
    }
  }
}
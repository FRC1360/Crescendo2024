// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ShintakeSubsystem extends SubsystemBase {

  private CANSparkFlex m_left;
  private CANSparkFlex m_right;
  private CANSparkMax m_back;

  public ShintakeSubsystem() {
    this.m_left = new CANSparkFlex(Constants.ShintakeConstants.LEFT_SHOOTAKE_CAN_ID, MotorType.kBrushless);
    this.m_right = new CANSparkFlex(Constants.ShintakeConstants.RIGHT_SHOOTAKE_CAN_ID, MotorType.kBrushless);
    this.m_back = new CANSparkMax(Constants.ShintakeConstants.BACK_SHOOTAKE_ID, MotorType.kBrushless);
  }
  
  //Moves both motors in the direction to intake when given a positive number
  //When not inverted, motors turn right
  public void intakePiece() {
    m_back.setInverted(false);
    m_back.set(Constants.ShintakeConstants.INTAKE_SPEED_BACK);
  }

  //Moves both motors in the direction to shoot when given a positive number
  public void shootSpeaker() {
    m_left.setInverted(true);
    m_right.setInverted(false);
    m_back.setInverted(false);
    m_left.set(Constants.ShintakeConstants.SHOOT_SPEED_FRONT);
    m_right.set(Constants.ShintakeConstants.SHOOT_SPEED_FRONT);
    m_back.set(Constants.ShintakeConstants.SHOOT_SPEED_BACK);
  }

  public void shootAmp() {
    m_left.setInverted(false);
    m_right.setInverted(true);
    m_back.setInverted(true);
    m_left.set(Constants.ShintakeConstants.SHOOT_SPEED_FRONT);
    m_right.set(Constants.ShintakeConstants.SHOOT_SPEED_FRONT);
    m_back.set(Constants.ShintakeConstants.SHOOT_SPEED_BACK);
  }

  //Stops both motors
  public void stop() {
    m_left.set(0.0);
    m_right.set(0.0);
    m_back.set(0.0);
  }

  public void varIntake(int backSpeed) {
    m_back.setInverted(true);
    m_back.set(backSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (m_left.get() == 0 && m_right.get() == 0 && m_back.get() == 0) {
      SmartDashboard.putBoolean("Intake in motion: ", false);
    }
    else {
      SmartDashboard.putBoolean("Intake in motion: ", true);
    }
  }
}
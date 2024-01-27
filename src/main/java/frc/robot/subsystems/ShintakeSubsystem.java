// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShintakeSubsystem extends SubsystemBase {

  private CANSparkMax m_left;
  private CANSparkMax m_right;
  private CANSparkMax m_back;
  private DigitalInput m_digital;
  private Counter m_counter;

  public ShintakeSubsystem() {
    //Using CANSparkFlexes for the two shooter neo vortexes
    this.m_left = new CANSparkMax(Constants.ShintakeConstants.LEFT_SHOOTAKE_CAN_ID, MotorType.kBrushless);
    this.m_right = new CANSparkMax(Constants.ShintakeConstants.RIGHT_SHOOTAKE_CAN_ID, MotorType.kBrushless);
    this.m_back = new CANSparkMax(Constants.ShintakeConstants.BACK_SHOOTAKE_ID, MotorType.kBrushless);
    this.m_digital = new DigitalInput(Constants.ShintakeConstants.SHINTAKE_SENSOR_PIN);
    this.m_counter = new Counter(m_digital);

    m_left.setInverted(false);
    m_right.setInverted(false);
    m_back.setInverted(true);
  }

  //Stops both motors
  public void stopShooter() {
    m_left.set(0.0);
    m_right.set(0.0);
  }

  public void stopIntake() {
    m_back.set(0.0);
  }

  public void varIntake(double backSpeed) {
    m_back.set(backSpeed);
  }

  public void varShoot(double speed) {
    //Set one to negative to side
    m_left.set(speed);
    m_right.set(-speed);
  }

  public boolean getDigitalInput() {
    return m_digital.get();
  }

  public int getShintakeCount() {
    return m_counter.get();
  }

  public void resetShintakeCount() {
    m_counter.reset();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake in motion: ", !(m_left.get() == 0 && m_right.get() == 0 && m_back.get() == 0));
    SmartDashboard.putBoolean("intake sensor state", m_digital.get());
  }
}
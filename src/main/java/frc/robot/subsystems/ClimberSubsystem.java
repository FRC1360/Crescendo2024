// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.OrbitPID;

public class ClimberSubsystem extends SubsystemBase {

	private final CANSparkMax m_climbMotorLead;
	private final CANSparkMax m_climbMotorSlave;
	private OrbitPID heightPID;
	private Boolean isExtended = false;

	public ClimberSubsystem() {
		this.m_climbMotorLead = new CANSparkMax(Constants.ClimbConstants.CLIMBER_LEAD_CAN_ID, MotorType.kBrushless);
		this.m_climbMotorSlave = new CANSparkMax(Constants.ClimbConstants.CLIMBER_SLAVE_CAN_ID, MotorType.kBrushless);
		;

		this.heightPID = new OrbitPID(0, 0, 0);
		/*
		 * SmartDashboard.putNumber("ki", 0);
		 * SmartDashboard.putNumber("kp", 0);
		 * SmartDashboard.putNumber("kd", 0);
		 */

		m_climbMotorSlave.follow(m_climbMotorLead);
	}

	public void setEncoderPosition(double pos) {
		m_climbMotorLead.getEncoder().setPosition(pos);
	}

	public void stopClimber() {
		m_climbMotorLead.set(0);
	}

	public double setPosition(double pos) {
		// We need the actual height from motor roations
		// We need the height of 0 in rotations
		SmartDashboard.putNumber("lead encoder value", m_climbMotorLead.getEncoder().getPosition());
		SmartDashboard.putNumber("target height", 0);
		double output = heightPID.calculate(0, m_climbMotorLead.getEncoder().getPosition());
		m_climbMotorLead.set(output);

		// instead of positon, subtract current height
		double last_err = pos - m_climbMotorLead.getEncoder().getPosition();
		return last_err;
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("is extended: ", isExtended);
	}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

	private final CANSparkMax m_climbMotorLead;
	private final CANSparkMax m_climbMotorSlave;
	private Boolean isSafe = true;
	private boolean isExtended = false;

	public ClimberSubsystem() {
		this.m_climbMotorLead = new CANSparkMax(Constants.ClimbConstants.CLIMBER_LEAD_CAN_ID, MotorType.kBrushless);
		this.m_climbMotorSlave = new CANSparkMax(Constants.ClimbConstants.CLIMBER_SLAVE_CAN_ID, MotorType.kBrushless);

		m_climbMotorSlave.follow(m_climbMotorLead);
	}

	public double getEncoderPosition() {
		return m_climbMotorLead.getEncoder().getPosition();
	}

	public void stopClimber() {
		m_climbMotorLead.set(0);
	}

	public void goToPosition(double pos, double speed) {
		m_climbMotorLead.set(speed);
		if (climbMotorReady(pos)) {
			stopClimber();
		}
	}

	public boolean climbMotorReady(double targetPos) {
		return getEncoderPosition() != 0 && Math.abs(getEncoderPosition()
		 - targetPos) <= 0.5; 
		 //this is in rotations so the deadband number is much smaller
		}


	public void setIsSafe(boolean isSafe) {
			this.isSafe = isSafe;
	}

		public boolean getIsSafe() {
			return this.isSafe;
	}

	public boolean getIsExtended() {
		return this.isExtended;
	}

	public void setIsExtended(boolean isExtended) {
		this.isExtended = isExtended;
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("is extended: ", isSafe);
	}
}
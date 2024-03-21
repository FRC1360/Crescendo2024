// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

	private final CANSparkMax m_climbMotorLead;
	private final CANSparkMax m_climbMotorSlave;
	private Boolean isSafe = true;
	private boolean isExtended = false;
	private double targetHeight = 0.0;

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

	public void raiseClimber() {
		m_climbMotorLead.set(Constants.ClimbConstants.LEAD_CLIMBER_MOTOR__SPEED);
	}

	public void lowerClimber() {
		m_climbMotorLead.set(-Constants.ClimbConstants.LEAD_CLIMBER_MOTOR__SPEED);
	}

	public void goToPosition() { // Rotations | Goes to the target position set with no limitations currently
		if (this.targetHeight > getEncoderPosition()) {
			m_climbMotorLead.set(Constants.ClimbConstants.LEAD_CLIMBER_MOTOR__SPEED);
		}
		if (this.targetHeight < getEncoderPosition()) {
			m_climbMotorLead.set(-Constants.ClimbConstants.LEAD_CLIMBER_MOTOR__SPEED);
		}
		if (climbMotorReady(this.targetHeight)) {
			stopClimber();
		}
	}

	public void setTargetHeight(double targetHeight) { // sets the target height for the climber to go to
		this.targetHeight = targetHeight;
	}

	public double gettargetHeight() {
		return this.targetHeight;
	}

	public boolean climbMotorReady(double targetPos) {
		return !(getEncoderPosition() < 0) && Math.abs(getEncoderPosition()
				- targetPos) <= 0.5;
		// this is in rotations so the deadband number is much smaller
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
	public void periodic() { // displays if it is extended, if it is safe, the current height in rotations,
								// and sets the position to the target Height
		SmartDashboard.putBoolean("Is extended ", isExtended);
		SmartDashboard.putBoolean("Is climber safe", isSafe);
		SmartDashboard.putNumber("Climber height in rotations", getEncoderPosition());
		// goToPosition();

	}
}
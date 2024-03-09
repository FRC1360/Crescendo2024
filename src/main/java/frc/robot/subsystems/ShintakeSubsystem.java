// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TuningTable;

public class ShintakeSubsystem extends SubsystemBase {

	private CANSparkFlex m_left;
	private CANSparkFlex m_right;
	private RelativeEncoder m_encoderLeft;
	private RelativeEncoder m_encoderRight;
	private CANSparkMax m_back;
	private DigitalInput m_digital;
	private Counter m_counter;
	private SparkPIDController rightWheelPID;
	private SparkPIDController leftWheelPID;
	private double leftVelocity, rightVelocity;

	private final TuningTable kP_Upper = new TuningTable("P_Upper_Shooter");
	private final TuningTable kI_Upper = new TuningTable("I_Upper_Shooter");
	private final TuningTable kD_Upper = new TuningTable("D_Upper_Shooter");
	private final TuningTable kF_Upper = new TuningTable("F_Upper_Shooter");
	private final TuningTable kP_Lower = new TuningTable("P_Lower_Shooter");
	private final TuningTable kI_Lower = new TuningTable("I_Lower_Shooter");
	private final TuningTable kD_Lower = new TuningTable("D_Lower_Shooter");
	private final TuningTable kF_Lower = new TuningTable("F_Lower_Shooter");

	private double leftkP, leftkI, leftkD, leftFF, rightkI, rightkP, rightkD, rightFF;

	public ShintakeSubsystem() {
		// Using CANSparkFlexes for the two shooter neo vortexes
		this.m_left = new CANSparkFlex(Constants.ShintakeConstants.LEFT_SHOOTAKE_CAN_ID, MotorType.kBrushless);
		this.m_right = new CANSparkFlex(Constants.ShintakeConstants.RIGHT_SHOOTAKE_CAN_ID, MotorType.kBrushless);
		this.m_back = new CANSparkMax(Constants.ShintakeConstants.BACK_SHOOTAKE_ID, MotorType.kBrushless);
		this.m_back.setIdleMode(IdleMode.kBrake);
		this.m_digital = new DigitalInput(Constants.ShintakeConstants.SHINTAKE_SENSOR_PIN);
		this.m_counter = new Counter(m_digital);
		this.m_encoderLeft = m_left.getEncoder();
		this.m_encoderRight = m_right.getEncoder();
		this.leftkP = 0.0008;
		this.leftkI = 0.000;
		this.leftkD = 0.00;
		this.leftFF = 0.00016;
		this.rightkP = 0.0008;
		this.rightkI = 0.0;
		this.rightkD = 0.00;
		this.rightFF = 0.00016;

		Preferences.initDouble("right kP", rightkP);
		Preferences.initDouble("right kI", rightkI);
		Preferences.initDouble("right kD", rightkD);
		Preferences.initDouble("right FF", rightFF);
		Preferences.initDouble("left kP", leftkP);
		Preferences.initDouble("left kI", leftkI);
		Preferences.initDouble("left kD", leftkD);
		Preferences.initDouble("left FF", leftFF);
		// kP_Upper.setDefault(Constants.Shooter.kP_Upper);
		// kI_Upper.setDefault(Constants.Shooter.kI_Upper);
		// kD_Upper.setDefault(Constants.Shooter.kD_Upper);
		// kF_Upper.setDefault(Constants.Shooter.kFF_Upper);

		// kP_Lower.setDefault(Constants.Shooter.kP_Lower);
		// kI_Lower.setDefault(Constants.Shooter.kI_Lower);
		// kD_Lower.setDefault(Constants.Shooter.kD_Lower);
		// kF_Lower.setDefault(Constants.Shooter.kFF_Lower);

		this.rightWheelPID = m_right.getPIDController();
		this.leftWheelPID = m_left.getPIDController();

		leftWheelPID.setP(leftkP); // 0.0011 (3 zeros, 1 one, 1 five)
		// leftWheelPID.setI(kI_Lower.get());
		// leftWheelPID.setD(0.0005); //0.00012
		leftWheelPID.setFF(leftFF); // 0.0001815 (3 zeros, 1 one, 1 eight, 1 one, 1 five)

		rightWheelPID.setP(rightkP); // 0.0009 (4 zeros, 1 three)
		// rightWheelPID.setI(kI_Upper.get());
		// rightWheelPID.setD(0.0005); // 0.0005 (3 zeros, 1 five)
		rightWheelPID.setFF(rightFF); // 0.0001807 (3 zeros, 1 one, 1 eight, 1 zero, 1 seven)

		m_left.setInverted(false);
		m_right.setInverted(true);
		m_back.setInverted(false);
	}

	public boolean isSwitchSet() {
		return m_counter.get() > 0;
	}

	public double getBackEncoder() {
		return m_back.getEncoder().getPosition();
	}

	public boolean getShooterReady(double prev) {
		varIntake(-Constants.ShintakeConstants.UNFEED_SPEED_BACK);
		if (-(getBackEncoder() - prev) > (58.095 - 57.548)) {
			stopIntake();
			return true;
		}
		return false;
	}

	public void initializeCounter() {
		m_counter.reset();
	}

	public void setVelocity(double rightVelocity, double leftVelocity) {
		System.out.println("Setting velocity to: " + rightVelocity);
		if (this.leftVelocity != leftVelocity)
		{
			this.leftVelocity = leftVelocity;
			leftWheelPID.setReference(this.leftVelocity, CANSparkFlex.ControlType.kVelocity);
		}
		
		if (this.rightVelocity != rightVelocity)
		{
			this.rightVelocity = rightVelocity;
			rightWheelPID.setReference(this.rightVelocity, CANSparkFlex.ControlType.kVelocity);	
		}
	}

	public boolean shooterWheelsReady() {
		return getVelocityLeft() != 0 && getVelocityRight() != 0
				&& Math.abs(getVelocityLeft() - this.leftVelocity) <= 100
				&& Math.abs(getVelocityRight() - this.rightVelocity) <= 100;
	}

	public double getVelocityLeft() {
		return m_encoderLeft.getVelocity();
	}

	public double getVelocityRight() {
		return m_encoderRight.getVelocity();
	}

	// Stops both motors
	public void stopShooter() {
		this.leftVelocity = 0.0; 
		this.rightVelocity = 0.0; 
		m_left.set(0.0);
		m_right.set(0.0);
	}

	public void stopIntake() {
		m_back.set(0.0);
	}

	public void varIntake(double backSpeed) {
		m_back.set(backSpeed);
	}

	public void varFix(double backSpeed) {
		m_back.set(-backSpeed);
	}

	public void varShoot(double speed) {
		// Set one to negative to side
		m_left.set(speed);
		m_right.set(speed);
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
		SmartDashboard.putBoolean("Intake in motion: ",
				!(m_left.get() == 0 && m_right.get() == 0 && m_back.get() == 0));
		SmartDashboard.putBoolean("intake sensor state", m_digital.get());
		SmartDashboard.putNumber("left velocity", m_encoderLeft.getVelocity());
		SmartDashboard.putNumber("right velocity", m_encoderRight.getVelocity());
		SmartDashboard.putNumber("Target left Wheel Velocity",
				Constants.ShintakeConstants.TARGET_SHOOT_VELOCITY_SPEAKER);
		SmartDashboard.putNumber("Target right Wheel Velocity",
				Constants.ShintakeConstants.TARGET_SHOOT_VELOCITY_SPEAKER);
		// SmartDashboard.putNumber("left Wheel kp", kp);
		// leftWheelPID.setP()
		// SmartDashboard.getNumber("right kP", rightkP);
		// rightWheelPID.setP(rightkP);
		// SmartDashboard.getNumber("right kI", rightkI);
		// rightWheelPID.setI(rightkI);
		// SmartDashboard.getNumber("right kD", rightkD);
		// rightWheelPID.setD(rightkD);
		// SmartDashboard.getNumber("left kP", leftkP);
		// leftWheelPID.setP(leftkP);
		// SmartDashboard.getNumber("left kI", leftkI);
		// leftWheelPID.setI(leftkI);
		// SmartDashboard.getNumber("left kD", leftkD);
		// leftWheelPID.setD(leftkD);
		Preferences.getDouble("right kP", rightkP);
		Preferences.getDouble("right kI", rightkI);
		Preferences.getDouble("right kD", rightkD);
		Preferences.getDouble("left kP", leftkP);
		Preferences.getDouble("left kI", leftkI);
		Preferences.getDouble("left kD", leftkD);
	}
}
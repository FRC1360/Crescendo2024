package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.OrbitPID;

public class ArmChassisPivotSubsystem extends SubsystemBase {
    
    private CANSparkMax ACPMotorMaster;
    private CANSparkMax ACPMotorSlave;
    private double targetAngle;

    // we can use SparkMAX integrated PID, its got more features and is easier to use
    public OrbitPID movePIDController;  // PID Controller for following Trapazoid Motion Profile
    // you shouldn't need separate motion profiles for each direction, PIDF will handle gravity and whatnot
    public TrapezoidProfile.Constraints ACPUpMotionProfileConstraints;
    public TrapezoidProfile.Constraints ACPDownMotionProfileConstraints; 

    private double angularVelocity;  // angular velocity in deg / second
    private double lastAngle;
    private long lastTime;

    private boolean transitioning;
    private double scheduledAngle;

    // this can be moved to the command, avoid handling operator input in subsystems
    private DoubleSupplier manualOffset;
    private BooleanSupplier manualOffsetEnable;

    private AnalogEncoder absoluteEncoder;

    public ArmFeedforward ACPFeedForward;  
    
    // try handling states using an enum, its more readable and effective
    private boolean inIntakePosition;
    private boolean isSafe; 

    public ArmChassisPivotSubsystem(DoubleSupplier manualOffset, BooleanSupplier manualOffsetEnable) {
        //this.holdPIDController = new OrbitPID(0.035, 0.0000075, 0.0); //kP = 0.045
        this.movePIDController = new OrbitPID(0.0632, 0.0, 0.0);  // kP = 0.02

        SmartDashboard.putNumber("ACPMoveKp", movePIDController.kP);
        SmartDashboard.putNumber("ACPMoveKi", movePIDController.kI);
        SmartDashboard.putNumber("ACPMoveKd", movePIDController.kD);

        // This units are deg / second for velocity and deg / sec^2 for acceleration
        this.ACPUpMotionProfileConstraints = new TrapezoidProfile.Constraints(200.0, 350.0); 
        this.ACPDownMotionProfileConstraints = new TrapezoidProfile.Constraints(100.0, 250.0);
        this.targetAngle = Constants.ACPConstants.HOME_POSITION_ACP;

        this.ACPMotorMaster = new CANSparkMax(Constants.ACPConstants.ACP_MOTOR_MASTER, MotorType.kBrushless);
        this.ACPMotorSlave = new CANSparkMax(Constants.ACPConstants.ACP_MOTOR_SLAVE, MotorType.kBrushless); 

        this.ACPFeedForward = new ArmFeedforward(0.0, 0.0005, 0.0); //kG = 0.001
        SmartDashboard.putNumber("ACPMoveKg", ACPFeedForward.kg);

        this.ACPMotorMaster.restoreFactoryDefaults();
        this.ACPMotorSlave.restoreFactoryDefaults();

        this.ACPMotorMaster.setIdleMode(IdleMode.kBrake);
        this.ACPMotorSlave.setIdleMode(IdleMode.kBrake);

        this.ACPMotorMaster.setSmartCurrentLimit(80);
        this.ACPMotorSlave.setSmartCurrentLimit(80);

        //this.ACPMotorSlave.follow(this.ACPMotorMaster);

        this.transitioning = false;
        this.scheduledAngle = Double.NaN;

        this.manualOffset = manualOffset;
        this.manualOffsetEnable = manualOffsetEnable;

        this.absoluteEncoder = new AnalogEncoder(Constants.ACPConstants.ACP_ENCODER);

        this.inIntakePosition = false;
        this.isSafe = true; 

        resetMotorRotations();   
    }

    public void checkSafety() { 
        this.isSafe = true; 
    }

    public double getMotorRotations() {
        return this.ACPMotorMaster.getEncoder().getPosition();
    }

    public double getACPAngle() {
        return this.rotationsToAngleConversion(this.getMotorRotations());
    }

    public void setACPSpeed(double speed) {
        if (this.getACPAngle() > Constants.ACPConstants.MAX_ACP_ANGLE
             || this.getACPAngle() < Constants.ACPConstants.MIN_ACP_ANGLE) 
                speed = 0.0; 
        
        this.ACPMotorMaster.set(-speed);
        this.ACPMotorSlave.set(-speed);
    }

    public void resetMotorRotations() {
        // 
        double newPos = -((absoluteEncoder.getAbsolutePosition() - Constants.ACPConstants.ACP_ENCODER_OFFSET) / Constants.ACPConstants.ACP_GEAR_RATIO);  
        
        SmartDashboard.putNumber("New_Pos", newPos);

        if(this.ACPMotorMaster.getEncoder().setPosition(newPos) == REVLibError.kOk) {
            System.out.println("Reset ACP Rotations");
            SmartDashboard.putBoolean("ACP_Encoder_Updated", true); 
        } else {
            System.out.println("Failed to reset ACP Rotations");
            SmartDashboard.putBoolean("ACP_Encoder_Updated", false); 
        }
        
    }

    /*
     * Sets arm voltage based off 0.0 - 12.0
     */
    public void setACPVoltage(double voltage) {
        if (this.getACPAngle() > Constants.ACPConstants.MAX_ACP_ANGLE
             || this.getACPAngle() < Constants.ACPConstants.MIN_ACP_ANGLE) 
                voltage = 0.0;
        
        this.ACPMotorMaster.setVoltage(voltage);
        this.ACPMotorSlave.setVoltage(voltage);
    }

    /*
     * Sets arm voltage based off 0.0 - 1.0 
     */
    public void setACPNormalizedVoltage(double voltage) {
        this.setACPVoltage(voltage * 12.0);  // Should probably change this to a constant somewhere for ARM_VOLTAGE
    }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public double getTargetAngle() {
        return this.targetAngle + (manualOffsetEnable.getAsBoolean() ? manualOffset.getAsDouble() : 0);
    }

    /*
     * Converts motor rotations  to angle (0 - 360)
     */
    public double rotationsToAngleConversion(double encoderPosition) {
        // encoderPosition * 360.0 = angle of motor rotation
        // angle of motor rotation * GEAR_RATIO = ACP angle
        // ACP angle % 360 = keep range between 0-360
        return (encoderPosition * 360.0 * Constants.ACPConstants.ACP_GEAR_RATIO);
    }

    private void updateAngularVelocity() {
        long currentTime = System.currentTimeMillis();

        double deltaTime = (currentTime - lastTime) / 1000.0;

        this.angularVelocity = (this.getACPAngle() - lastAngle) / deltaTime;
        this.lastAngle = this.getACPAngle();
        this.lastTime = currentTime;
    }

    public double getAngluarVelocity() {
        return this.angularVelocity;
    }

    public boolean atTarget() {
        return Math.abs(this.getTargetAngle() - this.getACPAngle()) <= 3.0;  // Should make this a constant
    }

    public BooleanSupplier inTransitionState() {
        return () -> this.transitioning;
    }

    public void setScheduledAngle(double angle) {
        this.scheduledAngle = angle;
    }

    public double getScheduledAngle() {
        return this.scheduledAngle;
    }

    // this should probably be a flag set by commands when they start moving the arm, not something that you check this way
    // this isnt very flexible and may give bad data near the target positions
    public void checkTransitioning() {
        transitioning = !(Math.abs(this.getACPAngle()) < 2) && (this.getScheduledAngle() > 0.0 && this.getACPAngle() < 0.0) || (this.getScheduledAngle() < 0.0 && this.getACPAngle() > 0.0);
    }

    // the shintake isnt part of this subsystem
    public boolean getIntakePosition() {
        return this.inIntakePosition;
    }

    public void setIntakePosition(boolean inIntakePosition) {
        this.inIntakePosition = inIntakePosition;
    }

    @Override
    public void periodic() {
        updateAngularVelocity();

        if(!transitioning)
            checkTransitioning();

        if(transitioning && this.atTarget() && (this.getScheduledAngle() == this.getTargetAngle())) {
            transitioning = false;
            this.setScheduledAngle(Double.NaN);
        }
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("ACP_Target_Angle", this.getTargetAngle());
        SmartDashboard.putNumber("ACP_Angle", this.getACPAngle());
        // SmartDashboard.putNumber("ACP_Manual_Offset", this.manualOffset.getAsDouble());
        // SmartDashboard.putNumber("ACP_Scheduled_Angle", this.getScheduledAngle());

        SmartDashboard.putNumber("ACP_Angular_Velocity", this.getAngluarVelocity());

        SmartDashboard.putNumber("ACP_Move_P_Gain", this.movePIDController.getPTerm());
        SmartDashboard.putNumber("ACP_Move_I_Gain", this.movePIDController.getITerm());
        SmartDashboard.putNumber("ACP_Move_D_Gain", this.movePIDController.getDTerm());

        // SmartDashboard.putBoolean("ACP_Transition_State", transitioning);

        SmartDashboard.putNumber("ACP_Absolute_Encoder_Get", this.absoluteEncoder.get());
        SmartDashboard.putNumber("ACP_Absolute_Encoder_Absolute", this.absoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("ACP_Motor_Encoder", this.ACPMotorMaster.getEncoder().getPosition());

        SmartDashboard.putNumber("ACP_Master_Current", this.ACPMotorMaster.getOutputCurrent());

        movePIDController.kP = SmartDashboard.getNumber("ACPMoveKp", movePIDController.kP);
        movePIDController.kI = SmartDashboard.getNumber("ACPMoveKi", movePIDController.kI);
        movePIDController.kD = SmartDashboard.getNumber("ACPMoveKd", movePIDController.kD);
        ACPFeedForward = new ArmFeedforward(0, SmartDashboard.getNumber("ACPMoveKg", ACPFeedForward.kg), 0);

        System.out.println(movePIDController.kP);
    }
}

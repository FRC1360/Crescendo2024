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

public class ShoulderSubsystem extends SubsystemBase {
    
    private CANSparkMax shoulderMotorMaster;
    private CANSparkMax shoulderMotorSlave;
    private double targetAngle;

    public OrbitPID holdPIDController;  // PID Controller for HoldToTarget
    public OrbitPID movePIDController;  // PID Controller for following Trapazoid Motion Profile
    public TrapezoidProfile.Constraints shoulderUpMotionProfileConstraints;
    public TrapezoidProfile.Constraints shoulderDownMotionProfileConstraints;


    private double angularVelocity;  // angular velocity in deg / second
    private double lastAngle;
    private long lastTime;

    private boolean transitioning;
    private double scheduledAngle;

    private DoubleSupplier manualOffset;
    private BooleanSupplier manualOffsetEnable;

    private AnalogEncoder absoluteEncoder;

    public ArmFeedforward shoulderFeedForward;  
    
    private boolean inIntakePosition;

    private boolean isSafe; 

    public ShoulderSubsystem(DoubleSupplier manualOffset, BooleanSupplier manualOffsetEnable) {
        this.holdPIDController = new OrbitPID(0.035, 0.0000075, 0.0); //kP = 0.045
        this.movePIDController = new OrbitPID(0.0632, 0.0, 0.0);  // kP = 0.02

        SmartDashboard.putNumber("ShoulderMoveKp", movePIDController.kP);
        SmartDashboard.putNumber("ShoulderMoveKi", movePIDController.kI);
        SmartDashboard.putNumber("ShoulderMoveKd", movePIDController.kD);

        // This units are deg / second for velocity and deg / sec^2 for acceleration
        this.shoulderUpMotionProfileConstraints = new TrapezoidProfile.Constraints(200.0, 350.0); 
        this.shoulderDownMotionProfileConstraints = new TrapezoidProfile.Constraints(100.0, 250.0); 
        this.targetAngle = Constants.HOME_POSITION_SHOULDER;

        this.shoulderMotorMaster = new CANSparkMax(Constants.SHOULDER_MOTOR_MASTER, MotorType.kBrushless);
        this.shoulderMotorSlave = new CANSparkMax(Constants.SHOULDER_MOTOR_SLAVE, MotorType.kBrushless); 

        this.shoulderFeedForward = new ArmFeedforward(0.0, 0.0005, 0.0); //kG = 0.001
        SmartDashboard.putNumber("ShoulderMoveKg", shoulderFeedForward.kg);

        this.shoulderMotorMaster.restoreFactoryDefaults();
        this.shoulderMotorSlave.restoreFactoryDefaults();

        this.shoulderMotorMaster.setIdleMode(IdleMode.kBrake);
        this.shoulderMotorSlave.setIdleMode(IdleMode.kBrake);

        this.shoulderMotorMaster.setSmartCurrentLimit(80);
        this.shoulderMotorSlave.setSmartCurrentLimit(80);

        //this.shoulderMotorSlave.follow(this.shoulderMotorMaster);

        this.transitioning = false;
        this.scheduledAngle = Double.NaN;

        this.manualOffset = manualOffset;
        this.manualOffsetEnable = manualOffsetEnable;

        this.absoluteEncoder = new AnalogEncoder(Constants.SHOULDER_ENCODER);

        this.inIntakePosition = false;
        this.isSafe = true; 

        resetMotorRotations();
        
    }

    public void checkSafety() { 
        this.isSafe = true; 
    }

    public double getMotorRotations() {
        return this.shoulderMotorMaster.getEncoder().getPosition();
    }

    public double getShoulderAngle() {
        return this.rotationsToAngleConversion(this.getMotorRotations());
    }

    public void setShoulderSpeed(double speed) {
        if (this.getShoulderAngle() > Constants.MAX_SHOULDER_ANGLE
             || this.getShoulderAngle() < Constants.MIN_SHOULDER_ANGLE) 
                speed = 0.0; 
        
        this.shoulderMotorMaster.set(-speed);
        this.shoulderMotorSlave.set(-speed);
    }

    public void resetMotorRotations() {
        // 
        double newPos = -((absoluteEncoder.getAbsolutePosition() - Constants.SHOULDER_ENCODER_OFFSET) / Constants.SHOULDER_GEAR_RATIO);  
        
        SmartDashboard.putNumber("New_Pos", newPos);

        if(this.shoulderMotorMaster.getEncoder().setPosition(newPos) == REVLibError.kOk) {
            System.out.println("Reset Shoulder Rotations");
            SmartDashboard.putBoolean("Shoulder_Encoder_Updated", true); 
        } else {
            System.out.println("Failed to reset Shoulder Rotations");
            SmartDashboard.putBoolean("Shoulder_Encoder_Updated", false); 
        }
        
    }

    /*
     * Sets arm voltage based off 0.0 - 12.0
     */
    public void setShoulderVoltage(double voltage) {
        if (this.getShoulderAngle() > Constants.MAX_SHOULDER_ANGLE
             || this.getShoulderAngle() < Constants.MIN_SHOULDER_ANGLE) 
                voltage = 0.0;
        
        this.shoulderMotorMaster.setVoltage(voltage);
        this.shoulderMotorSlave.setVoltage(voltage);
    }

    /*
     * Sets arm voltage based off 0.0 - 1.0 
     */
    public void setShoulderNormalizedVoltage(double voltage) {
        this.setShoulderVoltage(voltage * 12.0);  // Should probably change this to a constant somewhere for ARM_VOLTAGE
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
        // angle of motor rotation * GEAR_RATIO = shoulder angle
        // shoulder angle % 360 = keep range between 0-360
        return (encoderPosition * 360.0 * Constants.SHOULDER_GEAR_RATIO);
    }

    private void updateAngularVelocity() {
        long currentTime = System.currentTimeMillis();

        double deltaTime = (currentTime - lastTime) / 1000.0;

        this.angularVelocity = (this.getShoulderAngle() - lastAngle) / deltaTime;
        this.lastAngle = this.getShoulderAngle();
        this.lastTime = currentTime;
    }

    public double getAngluarVelocity() {
        return this.angularVelocity;
    }

    public boolean atTarget() {
        return Math.abs(this.getTargetAngle() - this.getShoulderAngle()) <= 3.0;  // Should make this a constant
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

    public void checkTransitioning() {
        transitioning = !(Math.abs(this.getShoulderAngle()) < 2) && (this.getScheduledAngle() > 0.0 && this.getShoulderAngle() < 0.0) || (this.getScheduledAngle() < 0.0 && this.getShoulderAngle() > 0.0);
    }

    public boolean getInIntakePosition() {
        return this.inIntakePosition;
    }

    public void setInIntakePosition(boolean inIntakePosition) {
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
        SmartDashboard.putNumber("Shoulder_Hold_P_Gain", this.holdPIDController.getPTerm());
        SmartDashboard.putNumber("Shoulder_Hold_I_Gain", this.holdPIDController.getITerm());
        SmartDashboard.putNumber("Shoulder_Hold_D_Gain", this.holdPIDController.getDTerm());

        SmartDashboard.putNumber("Shoulder_Target_Angle", this.getTargetAngle());
        SmartDashboard.putNumber("Shoulder_Angle", this.getShoulderAngle());
        // SmartDashboard.putNumber("Shoulder_Manual_Offset", this.manualOffset.getAsDouble());
        // SmartDashboard.putNumber("Shoulder_Scheduled_Angle", this.getScheduledAngle());

        SmartDashboard.putNumber("Shoulder_Angular_Velocity", this.getAngluarVelocity());

        // SmartDashboard.putNumber("Shoulder_Move_P_Gain", this.movePIDController.getPTerm());
        // SmartDashboard.putNumber("Shoulder_Move_I_Gain", this.movePIDController.getITerm());
        // SmartDashboard.putNumber("Shoulder_Move_D_Gain", this.movePIDController.getDTerm());

        // SmartDashboard.putBoolean("Shoulder_Transition_State", transitioning);

        SmartDashboard.putNumber("Shoulder_Absolute_Encoder_Get", this.absoluteEncoder.get());
        SmartDashboard.putNumber("Shoulder_Absolute_Encoder_Absolute", this.absoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Shoulder_Motor_Encoder", this.shoulderMotorMaster.getEncoder().getPosition());

        SmartDashboard.putNumber("Shoulder_Master_Current", this.shoulderMotorMaster.getOutputCurrent());

        movePIDController.kP = SmartDashboard.getNumber("ShoulderMoveKp", movePIDController.kP);
        movePIDController.kI = SmartDashboard.getNumber("ShoulderMoveKi", movePIDController.kI);
        movePIDController.kD = SmartDashboard.getNumber("ShoulderMoveKd", movePIDController.kD);
        shoulderFeedForward = new ArmFeedforward(0, SmartDashboard.getNumber("ShoulderMoveKg", shoulderFeedForward.kg), 0);
    }

    public class ShoulderWristMessenger {
        public double getShoulderAngle() {
            return ShoulderSubsystem.this.getShoulderAngle();
        } 

        public double getTargetAngle() {
            return ShoulderSubsystem.this.getTargetAngle();
        }
    }
}

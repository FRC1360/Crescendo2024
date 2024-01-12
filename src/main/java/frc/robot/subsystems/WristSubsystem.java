package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.subsystems.ShoulderSubsystem.ShoulderWristMessenger;
import frc.robot.util.OrbitPID;

public class WristSubsystem extends SubsystemBase {

    private CANSparkMax wristMotor;
    private double wristOffset;  // Angle offset for the shoulder, should really be called angle 

    public OrbitPID holdPIDController;
    public OrbitPID movePIDController;
    public ArmFeedforward wristFeedForward;
    public TrapezoidProfile.Constraints wristMotionProfileConstraints;

    //private ShoulderWristMessenger shoulderWristMessenger;

    private double cacheOffset;
    private DoubleSupplier manualOffset;
    private BooleanSupplier manualOffsetEnable;

    private AnalogEncoder absoluteEncoder;

    private double lastTime; 

    private Double lastAngle; 
    private Double angularVelocity; // degrees per second

    public WristSubsystem(DoubleSupplier manualOffset, BooleanSupplier manualOffsetEnable) {
        this.wristMotor = new CANSparkMax(Constants.WristConstants.WRIST_MOTOR, MotorType.kBrushless);
        
        this.wristOffset = 0.0;
        // kP = 0.0125
        this.holdPIDController = new OrbitPID(0.04, 0.00005, 0.0); // kI - 0.000005
        this.movePIDController = new OrbitPID(0.025, 0.000000, 0.4);  // TODO - Tune

        this.wristFeedForward = new ArmFeedforward(0.0, 0.125, 0.0); // ks, kg, kv
        this.wristMotionProfileConstraints = new TrapezoidProfile.Constraints(200.0, 600.0);  // TODO - Tune

        this.wristMotor.restoreFactoryDefaults();
        this.wristMotor.setIdleMode(IdleMode.kBrake);
        this.wristMotor.setInverted(true);

        this.cacheOffset = 0.0;

        this.manualOffset = manualOffset;
        this.manualOffsetEnable = manualOffsetEnable;

        this.absoluteEncoder = new AnalogEncoder(Constants.WristConstants.WRIST_ENCODER);

        this.lastTime = -1; 
        this.lastAngle = Double.NaN; 
        this.angularVelocity = Double.NaN; 

        resetMotorRotations();
    }

    public void setIdleMode(IdleMode mode) { 
        this.wristMotor.setIdleMode(mode); 
    }

    public void resetMotorRotations() {
        double newPos = (this.absoluteEncoder.getAbsolutePosition() - Constants.WristConstants.WRIST_ENCODER_OFFSET) / Constants.WristConstants.WRIST_GEAR_RATIO;

        if(this.wristMotor.getEncoder().setPosition(newPos) == REVLibError.kOk) {
            System.out.println("Reset Shoulder Rotations to 0");
        } else {
            System.out.println("Failed to reset Shoulder Rotations");
        }
        
    }


    public double getMotorRotations() {
        return this.wristMotor.getEncoder().getPosition();
    }

    
    // Returns the wrist GLOBAL angle. The global angle is the angle relative to the shoulder
    public double getWristAngle() {
        return this.encoderToAngleConversion(this.getMotorRotations());
    }

    public void setWristSpeed(double speed) {
        if (this.getWristAngle() > Constants.WristConstants.WRIST_MAX_ANGLE
             || this.getWristAngle() < Constants.WristConstants.WRIST_MIN_ANGLE)
             speed = 0.0; 
        this.wristMotor.set(speed);
    }

    /*
     * Sets arm voltage based off 0.0 - 12.0
     */
    public void setWristVoltage(double voltage) {
        if (this.getWristAngle() > Constants.WristConstants.WRIST_MAX_ANGLE
            || this.getWristAngle() < Constants.WristConstants.WRIST_MIN_ANGLE)
                voltage = 0.0; 
        this.wristMotor.setVoltage(voltage);
    }

    /*
     * Sets arm voltage based off 0.0 - 1.0 
     */
    public void setWristNormalizedVoltage(double voltage) {
        this.setWristVoltage(voltage * 12.0);  // Should probably change this to a constant somewhere for ARM_VOLTAGE
    }

    // This return a GLOBAL angle. The global angle is the angle relative to the shoulder
    /*public double getTargetAngle() {  // Use getTargetAngle() when doing commands to move the wrist
        
        return -this.shoulderWristMessenger.getShoulderAngle() + this.getWristOffset() + (manualOffsetEnable.getAsBoolean() ? -manualOffset.getAsDouble() : 0.0);
    }*/
 
    public void setWristOffset(double offset) {
        this.wristOffset = offset;
    }

    // The offset is more akin to a LOCAL angle. The local angle is the angle relative to wrist starting position.
    // This angle ensures the wrist doesn't move relative to the starting location while the shoulder rotates
    // For example, if the wrist starts pointing directly up and the wrist offset is 0, the wrist will stay pointing up
    // regardless of the shoulder's orientation.  Change this value when you want to change the angle of the wrist
    public double getWristOffset() {
        return this.wristOffset;
    }

    // The CACHE is a means of saving an arbitrary wrist position to return to later
    // Setting the cache offset saves the current angle
    // Getting the cache offset gets the angle that the wrist was at when it was last set
    public void setCacheOffset() {
        System.out.println("Setting cache offset to " + this.getWristOffset());
        this.cacheOffset = this.getWristOffset();
    }

    public double getCacheOffset() {
        return this.cacheOffset;
    }

    /*
     * Converts motor rotations to angle (0 - 360)
     */
    public double encoderToAngleConversion(double encoderPosition) {
        return (encoderPosition * 360.0 * 2.0 * Constants.WristConstants.WRIST_GEAR_RATIO);
    }

    public void updateSmartDashboard() {
        // SmartDashboard.putNumber("Wrist_Hold_P_Gain", this.holdPIDController.getPTerm());
        // SmartDashboard.putNumber("Wrist_Hold_I_Gain", this.holdPIDController.getITerm());
        // SmartDashboard.putNumber("Wrist_Hold_D_Gain", this.holdPIDController.getDTerm());

        // SmartDashboard.putNumber("Wrist_Move_P_Gain", this.movePIDController.getPTerm());
        // SmartDashboard.putNumber("Wrist_Move_I_Gain", this.movePIDController.getITerm());
        // SmartDashboard.putNumber("Wrist_Move_D_Gain", this.movePIDController.getDTerm());

        // SmartDashboard.putNumber("Wrist_Target_Angle", this.getTargetAngle());
        SmartDashboard.putNumber("Wrist_Angle", this.getWristAngle());
        SmartDashboard.putNumber("Wrist_NEO_Encoder", this.getMotorRotations()); 
        SmartDashboard.putNumber("Wrist_Motor_Rotations", this.getMotorRotations());
        // SmartDashboard.putNumber("Wrist_Cache_Offset", this.getCacheOffset());
        // SmartDashboard.putNumber("Wrist_Manual_Offset", this.manualOffset.getAsDouble());
        SmartDashboard.putNumber("Wrist_Offset", this.getWristOffset());
        SmartDashboard.putNumber("Wrist_Absolute_Encoder_Relative", this.absoluteEncoder.get());
        SmartDashboard.putNumber("Wrist_Absolute_Encoder_Absolute", this.absoluteEncoder.getAbsolutePosition());

        SmartDashboard.putNumber("Wrist_Angular_Velocity", this.getAngularVelocity().doubleValue()); 
    }

    public void updateAngularVelocity() { 
        double currentTime = (System.currentTimeMillis() / 1000.0); 
        double currentAngle = this.getWristAngle(); 

        if (this.lastTime != -1 && !this.lastAngle.isNaN()) {
            double deltaTime = currentTime - this.lastTime; 

            double deltaAngle = currentAngle - this.lastAngle.doubleValue(); 

            this.angularVelocity = deltaAngle / ((double) deltaTime); 
        }
        this.lastAngle = currentAngle; 
        this.lastTime = currentTime; 
    }

    public Double getAngularVelocity() {
        return this.angularVelocity;  
    }

    @Override
    public void periodic() { 
        updateAngularVelocity();
    }
    
}

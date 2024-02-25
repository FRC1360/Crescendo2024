package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.OrbitPID;
import frc.robot.util.OrbitTimer;

public class ShintakePivotSubsystem extends SubsystemBase {

    private CANSparkMax ShintakePivotMotor;
    private CANSparkMax ShintakePivotFollowingMotor;
    //private double ShintakePivotOffset; // Angle offset for the shoulder, should really be called angle
    private double targetAngle;

    public PIDController movePIDController;
    public ArmFeedforward ShintakePivotFeedForward;
    public TrapezoidProfile.Constraints ShintakePivotMotionProfileConstraints;

    private TrapezoidProfile stpMotionProfile;
    private TrapezoidProfile.State motionProfileStartState; 
    private TrapezoidProfile.State motionProfileEndState;  

    //private double cacheOffset;

    private DutyCycleEncoder absoluteEncoder;

    private double lastTime;

    private Double lastAngle;
    private Double angularVelocity; // degrees per second

    private OrbitTimer timer; 

    public ShintakePivotSubsystem() {
        
        this.ShintakePivotMotor = new CANSparkMax(Constants.STPConstants.ShintakePivot_MOTOR, MotorType.kBrushless);
        this.ShintakePivotFollowingMotor = new CANSparkMax(Constants.STPConstants.ShintakePivot_FOLLOW_MOTOR,
                MotorType.kBrushless);
        //this.ShintakePivotOffset = 0.0;
        this.ShintakePivotFollowingMotor.follow(ShintakePivotMotor);
        this.movePIDController = new PIDController(0.025, 0.0, 0.4); // TODO - Tune

        this.ShintakePivotFeedForward = new ArmFeedforward(0.0, 0.125, 0.0); // ks, kg, kv
        this.ShintakePivotMotionProfileConstraints = new TrapezoidProfile.Constraints(200.0, 600.0); // TODO - Tune
        this.stpMotionProfile = new TrapezoidProfile(this.ShintakePivotMotionProfileConstraints);

        this.motionProfileEndState = new TrapezoidProfile.State(Constants.HOME_POSITION_STP, 0.0);
        this.motionProfileStartState = new TrapezoidProfile.State(Constants.HOME_POSITION_STP, 0.0);

        this.ShintakePivotMotor.restoreFactoryDefaults();
        this.ShintakePivotMotor.setIdleMode(IdleMode.kBrake);
        this.ShintakePivotMotor.setInverted(true);

        this.targetAngle = Constants.HOME_POSITION_STP;

        this.timer = new OrbitTimer(); 

        //this.cacheOffset = 0.0;

        this.absoluteEncoder = new DutyCycleEncoder(Constants.STPConstants.ShintakePivot_ENCODER_CHANNEL);

        this.lastTime = -1;
        this.lastAngle = Double.NaN;
        this.angularVelocity = 0.0; //Double.NaN;

        Preferences.initDouble("ShintakePivot_Move_P_Gain", this.movePIDController.getP());
        Preferences.initDouble("ShintakePivot_Move_I_Gain", this.movePIDController.getI());
        Preferences.initDouble("ShintakePivot_Move_D_Gain", this.movePIDController.getD());
        Preferences.initDouble("ShintakePivot_Angle", this.getShintakePivotAngle());
        Preferences.initDouble("ShintakePivot_NEO_Encoder", this.getMotorRotations());
        Preferences.initDouble("ShintakePivot_Motor_Rotations", this.getMotorRotations());
        Preferences.initDouble("ShintakePivot_Absolute_Encoder_Relative", this.absoluteEncoder.get());
        Preferences.initDouble("ShintakePivot_Absolute_Encoder_Absolute", this.absoluteEncoder.getAbsolutePosition());
        Preferences.initDouble("ShintakePivot_Angular_Velocity", this.getAngularVelocity().doubleValue());


        resetMotorRotations();
    }

    public void setIdleMode(IdleMode mode) {
        this.ShintakePivotMotor.setIdleMode(mode);
    }

    public void resetMotorRotations() {
        double newPos = (this.absoluteEncoder.getAbsolutePosition()
                - Constants.STPConstants.ShintakePivot_ENCODER_OFFSET)
                / Constants.STPConstants.ShintakePivot_GEAR_RATIO;

        if (this.ShintakePivotMotor.getEncoder().setPosition(newPos) == REVLibError.kOk) {
            System.out.println("Reset Shoulder Rotations to 0");
        } else {
            System.out.println("Failed to reset Shoulder Rotations");
        }

    }

    public double getMotorRotations() {
        return this.ShintakePivotMotor.getEncoder().getPosition();
    }

    // Returns the ShintakePivot GLOBAL angle. The global angle is the angle
    // relative to the shoulder
    public double getShintakePivotAngle() {
        return this.encoderToAngleConversion(this.getMotorRotations());
    }

    public void setShintakePivotSpeed(double speed) {
        // if (this.getShintakePivotAngle() > Constants.STPConstants.ShintakePivot_MAX_ANGLE
        //         || this.getShintakePivotAngle() < Constants.STPConstants.ShintakePivot_MIN_ANGLE)
        //     speed = 0.0;
        this.ShintakePivotMotor.set(speed);
    }

    /*
     * Sets arm voltage based off 0.0 - 12.0
     */
    public void setShintakePivotVoltage(double voltage) {
        // if (this.getShintakePivotAngle() > Constants.STPConstants.ShintakePivot_MAX_ANGLE
        //         || this.getShintakePivotAngle() < Constants.STPConstants.ShintakePivot_MIN_ANGLE)
        //     voltage = 0.0;
        this.ShintakePivotMotor.setVoltage(voltage);
    }

    /*
     * Sets arm voltage based off 0.0 - 1.0
     */
    public void setShintakePivotNormalizedVoltage(double voltage) {
        this.setShintakePivotVoltage(voltage * 12.0); // Should probably change this to a constant somewhere for
                                                      // ARM_VOLTAGE
    }

    // This return a GLOBAL angle. The global angle is the angle relative to the
    // shoulder
    /*
     * public double getTargetAngle() { // Use getTargetAngle() when doing commands
     * to move the ShintakePivot
     * 
     * return -this.shoulderShintakePivotMessenger.getShoulderAngle() +
     * this.getShintakePivotOffset() + (manualOffsetEnable.getAsBoolean() ?
     * -manualOffset.getAsDouble() : 0.0);
     * }
     */

    // public void setShintakePivotOffset(double offset) {
    //     this.ShintakePivotOffset = offset;
    // }

    // The offset is more akin to a LOCAL angle. The local angle is the angle
    // relative to ShintakePivot starting position.
    // This angle ensures the ShintakePivot doesn't move relative to the starting
    // location while the shoulder rotates
    // For example, if the ShintakePivot starts pointing directly up and the
    // ShintakePivot offset is 0, the ShintakePivot will stay pointing up
    // regardless of the shoulder's orientation. Change this value when you want to
    // change the angle of the ShintakePivot
    // public double getShintakePivotOffset() {
    //     return this.ShintakePivotOffset;
    // }

    // The CACHE is a means of saving an arbitrary ShintakePivot position to return
    // to later
    // Setting the cache offset saves the current angle
    // Getting the cache offset gets the angle that the ShintakePivot was at when it
    // was last set
    // public void setCacheOffset() {
    //     System.out.println("Setting cache offset to " + this.getShintakePivotOffset());
    //     this.cacheOffset = this.getShintakePivotOffset();
    // }

    // public double getCacheOffset() {
    //     return this.cacheOffset;
    // }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;

        this.movePIDController.reset();

        this.motionProfileStartState = new TrapezoidProfile.State(this.getShintakePivotAngle(), this.getAngularVelocity());
        this.motionProfileEndState = new TrapezoidProfile.State(targetAngle, 0.0); 
        this.timer.start(); 

        System.out.println("Target angle for ACP scheduled for: " + targetAngle); 
    }

    public double calculateControlLoopOutput() { 
        // Motion profile outputs goal when finished
        TrapezoidProfile.State profileTarget = this.stpMotionProfile.calculate(this.timer.getTimeDeltaSec(), this.motionProfileEndState,
                this.motionProfileStartState);

        double target = profileTarget.position; 
        double input = this.getShintakePivotAngle(); 

        double pidOut = this.movePIDController.calculate(input, target); 

        double feedforwardOutput = this.ShintakePivotFeedForward.calculate(
            Math.toRadians(profileTarget.position),
            Math.toRadians(this.getAngularVelocity()));

        return pidOut + feedforwardOutput; 
    }
    

    public double getTargetAngle(){
        return this.targetAngle;
    }

    public boolean atTarget() { 
        return Math.abs(this.getTargetAngle() - this.getShintakePivotAngle()) <= Constants.STPConstants.STP_GO_TO_POS_TOLERANCE; 
    }

    /*
     * Converts motor rotations to angle (0 - 360)
     */
    public double encoderToAngleConversion(double encoderPosition) {
        return (encoderPosition * 360.0 * 2.0 * Constants.STPConstants.ShintakePivot_GEAR_RATIO);
    }

    public void updateSmartDashboard() {

        Preferences.getDouble("ShintakePivot_Move_P_Gain", this.movePIDController.getP());
        Preferences.getDouble("ShintakePivot_Move_I_Gain", this.movePIDController.getI());
        Preferences.getDouble("ShintakePivot_Move_D_Gain", this.movePIDController.getD());
        Preferences.getDouble("ShintakePivot_Angle", this.getShintakePivotAngle());
        Preferences.getDouble("ShintakePivot_NEO_Encoder", this.getMotorRotations());
        Preferences.getDouble("ShintakePivot_Motor_Rotations", this.getMotorRotations());
        Preferences.getDouble("ShintakePivot_Absolute_Encoder_Relative", this.absoluteEncoder.get());
        Preferences.getDouble("ShintakePivot_Absolute_Encoder_Absolute", this.absoluteEncoder.getAbsolutePosition());
        Preferences.getDouble("ShintakePivot_Angular_Velocity", this.getAngularVelocity().doubleValue());

        // SmartDashboard.putNumber("ShintakePivot_Move_P_Gain", this.movePIDController.getPTerm());
        // SmartDashboard.putNumber("ShintakePivot_Move_I_Gain", this.movePIDController.getITerm());
        // SmartDashboard.putNumber("ShintakePivot_Move_D_Gain", this.movePIDController.getDTerm());

        // SmartDashboard.putNumber("ShintakePivot_Angle", this.getShintakePivotAngle());
        // SmartDashboard.putNumber("ShintakePivot_NEO_Encoder", this.getMotorRotations());
        // SmartDashboard.putNumber("ShintakePivot_Motor_Rotations", this.getMotorRotations());
        // // SmartDashboard.putNumber("ShintakePivot_Cache_Offset",
        // // this.getCacheOffset());
        // // SmartDashboard.putNumber("ShintakePivot_Manual_Offset",
        // // this.manualOffset.getAsDouble());
        // //SmartDashboard.putNumber("ShintakePivot_Offset", this.getShintakePivotOffset());
        // SmartDashboard.putNumber("ShintakePivot_Absolute_Encoder_Relative", this.absoluteEncoder.get());
        // SmartDashboard.putNumber("ShintakePivot_Absolute_Encoder_Absolute", this.absoluteEncoder.getAbsolutePosition());

        // // movePIDController.kP = SmartDashboard.getNumber("STPMoveKp", movePIDController.kP);
        // // movePIDController.kI = SmartDashboard.getNumber("STPMoveKi", movePIDController.kI);
        // // movePIDController.kD = SmartDashboard.getNumber("STPMoveKd", movePIDController.kD);

        // SmartDashboard.putNumber("ShintakePivot_Angular_Velocity", this.getAngularVelocity().doubleValue());
    }

    public void updateAngularVelocity() {
        double currentTime = (System.currentTimeMillis() / 1000.0);
        double currentAngle = this.getShintakePivotAngle();

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
        updateSmartDashboard(); 
        
        double out = calculateControlLoopOutput(); 
        SmartDashboard.putNumber("STP_Control_Loop_Out", out); 
        this.setShintakePivotNormalizedVoltage(out);
    }

}

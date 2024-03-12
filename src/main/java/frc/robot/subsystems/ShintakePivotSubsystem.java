package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.OrbitPID;
import frc.robot.util.OrbitTimer;

public class ShintakePivotSubsystem extends SubsystemBase {

    private CANSparkMax STPMotorMaster;
    private CANSparkMax STPMotorSlave;
    private double targetAngle;

    public PIDController movePIDController;
    
    private TrapezoidProfile stpMotionProfile;
    private TrapezoidProfile.State motionProfileStartState; 
    private TrapezoidProfile.State motionProfileEndState; 

    private OrbitTimer timer;
    
    public TrapezoidProfile.Constraints STPMotionProfileConstraints;
    
    private Double angularVelocity;
    private double lastTime;
    private Double lastAngle;
    

    private double cacheOffset = 0.0;

    private DutyCycleEncoder absoluteEncoder;
    
     // degrees per second
     public ArmFeedforward STPFeedForward;

    private double STPOffset = Constants.STPConstants.STP_ENCODER_OFFSET;

    public InterpolatingDoubleTreeMap shintakePivotDistanceAngleMap; 

    private double kP = 0.025;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kS = 0.0;
    private double kG = 0.0;
    private double kV = 0.0;

    public ShintakePivotSubsystem() {
        this.movePIDController = new PIDController(kP, kI, kD); // TODO - Tune || 0.025, 0.0, 0.4

        this.STPMotorMaster = new CANSparkMax(Constants.STPConstants.STP_MOTOR_MASTER, MotorType.kBrushless);
        this.STPMotorSlave = new CANSparkMax(Constants.STPConstants.STP_MOTOR_SLAVE, MotorType.kBrushless);

        // this.STPFeedForward = new ArmFeedforward(0.0, 0.0, 0.0); // ks, kg, kv ||
        this.STPFeedForward = new ArmFeedforward(kS, kG, kV); 
        SmartDashboard.putNumber("STPMoveKg", STPFeedForward.kg);


        this.STPMotorMaster.restoreFactoryDefaults();
        this.STPMotorSlave.restoreFactoryDefaults();


        this.STPMotorMaster.setIdleMode(IdleMode.kBrake);
        this.STPMotorSlave.setIdleMode(IdleMode.kBrake);

        this.STPMotorMaster.setSmartCurrentLimit(80);
        this.STPMotorSlave.setSmartCurrentLimit(80);

        this.STPMotorSlave.follow(this.STPMotorMaster);

        this.STPMotorMaster.setInverted(true);
        //this.STPMotorSlave.setInverted(true);

        

        this.STPMotorMaster.getEncoder().setPositionConversionFactor(Constants.STPConstants.STP_GEAR_RATIO);

        this.absoluteEncoder = new DutyCycleEncoder(Constants.STPConstants.STP_ENCODER_CHANNEL);


        this.STPMotionProfileConstraints = new TrapezoidProfile.Constraints(180, 100); // TODO - Tune
        this.stpMotionProfile = new TrapezoidProfile(this.STPMotionProfileConstraints);

        //this.cacheOffset = 0.0;

        // I'm unsure if this is needed
        this.lastTime = -1;
        this.lastAngle = Double.NaN;
        this.angularVelocity = 0.0; //Double.NaN;

        Preferences.initDouble("STP_Move_P_Gain", this.movePIDController.getP());
        Preferences.initDouble("STP_Move_I_Gain", this.movePIDController.getI());
        Preferences.initDouble("STP_Move_D_Gain", this.movePIDController.getD());
        Preferences.initDouble("Shintake Pivot FeedForward kS", kS);
        Preferences.initDouble("Shintake Pivot FeedForward kG", kG);
        Preferences.initDouble("Shintake Pivot FeedForward kV", kV);

        resetMotorRotations();

        this.timer = new OrbitTimer();
        this.motionProfileStartState = new TrapezoidProfile.State(this.getSTPAngle(), 0.0); //this.getSTPAngle(), 0.0);
        this.motionProfileEndState = new TrapezoidProfile.State(this.getSTPAngle(), 0.0);

        this.targetAngle = this.getSTPAngle(); 
        
        this.shintakePivotDistanceAngleMap = new InterpolatingDoubleTreeMap(); 
        // Calc by inverse tan((2.045-0.9017)/(dist-0.248)) <-- target speaker height - height of shooter / dist - offset of speaker from center of bot
        this.shintakePivotDistanceAngleMap.put(0.5, 90-77.56); 
        this.shintakePivotDistanceAngleMap.put(1.0, 90-56.67); 
        this.shintakePivotDistanceAngleMap.put(1.5, 90-42.40); 
        this.shintakePivotDistanceAngleMap.put(2.0, 90-33.127); 
        this.shintakePivotDistanceAngleMap.put(2.5, 90-26.916); 
        this.shintakePivotDistanceAngleMap.put(3.0, 90-22.56); 

    }

    public double getMotorRotations() {
        return this.STPMotorMaster.getEncoder().getPosition();
    }

    public void setCacheOffset(double offset) { 
        this.cacheOffset += offset;
    }

    // Returns the ShintakePivot GLOBAL angle. The global angle is the angle
    // relative to the shoulder
    public double getSTPAngle() {
        return this.rotationsToAngleConversion(this.getMotorRotations());
    }

    public void setSTPSpeed(double speed) {
        if (this.getSTPAngle() > Constants.STPConstants.STP_MAX_ANGLE
                || this.getSTPAngle() < Constants.STPConstants.STP_MIN_ANGLE)
            speed = 0.0;
        this.STPMotorMaster.set(speed);
    }

    public void resetMotorRotations() {
        if (this.absoluteEncoder.getAbsolutePosition() == 0.0) { 
            DriverStation.reportError("STP Absolute encoder reports 0.0! Possibly not connected properly!", true);
            System.exit(1);
        }
        double newPos = (this.absoluteEncoder.getAbsolutePosition()- this.STPOffset);

        SmartDashboard.putNumber("New Pos", newPos);

        if (this.STPMotorMaster.getEncoder().setPosition(newPos) == REVLibError.kOk) {
            //System.out.println("Reset STP Rotations");
            SmartDashboard.putBoolean("STP_Encoder_Updated", true);
        } else {
            System.out.println("Failed to reset STP Rotations");
            SmartDashboard.putBoolean("STP_Encoder_Updated", false);
        }

    }

    public void setIdleMode(IdleMode mode) {
        this.STPMotorMaster.setIdleMode(mode);
    }

    /*
     * Sets arm voltage based off 0.0 - 12.0
     */
    public void setSTPVoltage(double voltage) {
        if (this.getSTPAngle() > Constants.STPConstants.STP_MAX_ANGLE
                || this.getSTPAngle() < Constants.STPConstants.STP_MIN_ANGLE)
            voltage = 0.0;
        this.STPMotorMaster.setVoltage(voltage);
    }

    /*
     * Sets arm voltage based off 0.0 - 1.0
     */
    public void setSTPNormalizedVoltage(double voltage) {
        this.setSTPVoltage(voltage * 12.0); // Should probably change this to a constant somewhere for
                                                      // ARM_VOLTAGE
    }

    // This return a GLOBAL angle. The global angle is the angle relative to the
    // shoulder
    /*
     * public double getTargetAngle() { // Use getTargetAngle() when doing commands
     * to move the shintake pivot
     * 
     * return -this.shoulderSTPMessenger.getShoulderAngle() +
     * this.getSTPOffset() + (manualOffsetEnable.getAsBoolean() ?
     * -manualOffset.getAsDouble() : 0.0);
     * }
     */

    // public void setSTPOffset(double offset) {
    //     this.STPOffset = offset;
    // }

    // The offset is more akin to a LOCAL angle. The local angle is the angle
    // relative to shintake pivot starting position.
    // This angle ensures the Shintake pivot doesn't move relative to the starting
    // location while the shoulder rotates
    // For example, if the ShintakePivot starts pointing directly up and the
    // ShintakePivot offset is 0, the ShintakePivot will stay pointing up
    // regardless of the shoulder's orientation. Change this value when you want to
    // change the angle of the ShintakePivot
    // public double getSTPOffset() {
    //     return this.STPOffset;
    // }

    // The CACHE is a means of saving an arbitrary ShintakePivot position to return
    // to later
    // Setting the cache offset saves the current angle
    // Getting the cache offset gets the angle that the ShintakePivot was at when it
    // was last set
    // public void setCacheOffset() {
    //     System.out.println("Setting cache offset to " + this.getSTPOffset());
    //     this.cacheOffset = this.getSTPOffset();
    // }

    // public double getCacheOffset() {
    //     return this.cacheOffset;
    // }

    public void setTargetAngle(double targetAngle) { // Degrees | Sets the target angle for the STP to go to | Min is -180 and Max is 360
        if (this.targetAngle == targetAngle)
        {
            return;
        }
        else if (this.targetAngle >= Constants.STPConstants.STP_MAX_ANGLE || this.targetAngle <= Constants.STPConstants.STP_MIN_ANGLE) { 
            DriverStation.reportError("Tried to set STP to above or below max or min; Target: " +  this.targetAngle, true);
            System.exit(1); 
        }
        this.targetAngle = targetAngle;

        this.movePIDController.reset();

        this.motionProfileStartState = new TrapezoidProfile.State(this.getSTPAngle(), this.getAngularVelocity());
        this.motionProfileEndState = new TrapezoidProfile.State(targetAngle, 0.0); 
        this.timer.start(); 

        System.out.println("Target angle for STP scheduled for: " + targetAngle); 
    }



    public double getTargetAngle(){
        return this.targetAngle;
    }

    

    /*
     * Converts motor rotations to angle (0 - 360)
     */
    public double rotationsToAngleConversion(double encoderPosition) {
        // encoderPosition * 360.0 = angle of motor rotation
        // angle of motor rotation * GEAR_RATIO = ACP angle
        // ACP angle % 360 = keep range between 0-360
        return (encoderPosition * 360.0) % 360;
    }

   

    public void updateAngularVelocity() {
        //time in seconds
        double currentTime = (System.currentTimeMillis() / 1000.0);
        double currentAngle = this.getSTPAngle();

        // if (this.lastTime != -1 && !this.lastAngle.isNaN()) {
        //     double deltaTime = currentTime - this.lastTime;

        //     double deltaAngle = currentAngle - this.lastAngle.doubleValue();

        //     this.angularVelocity = deltaAngle / ((double) deltaTime);
        // }
        
        double deltaTime = (currentTime - lastTime); /// 1000.0;

        this.angularVelocity = (currentAngle - lastAngle) / deltaTime;
        this.lastAngle = currentAngle;
        this.lastTime = currentTime;
    }

    public Double getAngularVelocity() {
        return this.angularVelocity;
    }

    public boolean atTarget() { 
            return Math.abs(this.getTargetAngle() - this.getSTPAngle()) <= Constants.STPConstants.STP_GO_TO_POS_TOLERANCE; 
    }

    public void resetEncoderOffset(){
        this.STPOffset = this.absoluteEncoder.getAbsolutePosition();
    }

    public double calculateControlLoopOutput() { 
        // Motion profile outputs goal when finished
        TrapezoidProfile.State profileTarget = this.stpMotionProfile.calculate(this.timer.getTimeDeltaSec(), this.motionProfileStartState,
                this.motionProfileEndState);

        double target = profileTarget.position; 
        double input = this.getSTPAngle(); 

        SmartDashboard.putNumber("STP_Profile_Position", target);
        SmartDashboard.putNumber("STP_Profile_Velocity", profileTarget.velocity);

        double pidOut = this.movePIDController.calculate(input, target); 

        double feedforwardOutput = this.STPFeedForward.calculate(
            Math.toRadians(profileTarget.position),
            Math.toRadians(this.getAngularVelocity()));

        return pidOut; //+ feedforwardOutput; 
    }
    
    @Override
    public void periodic() { // Displays angles, PID, and FF values for STP (also updates angular Velocity)
        updateAngularVelocity();
        updateSmartDashboard(); 
            

        // All of Control Loop motion is done within the subsystem -- simply set a target angle and the subsystem will go there
        // When the motion profile is finished, the result which it outputs will be the goal, making it a PID/FF control loop only
        double out = calculateControlLoopOutput(); 
        SmartDashboard.putNumber("STP_Control_Loop_Out", out); 
        this.setSTPNormalizedVoltage(out);
        // SmartDashboard.putNumber("Current Angle: ", this.getACPAngle());
        // SmartDashboard.putNumber("Target Angle: ", true);
    }
    public void updateSmartDashboard() {

        Preferences.getDouble("STP_Move_P_Gain", this.movePIDController.getP());
        Preferences.getDouble("STP_Move_I_Gain", this.movePIDController.getI());
        Preferences.getDouble("STP_Move_D_Gain", this.movePIDController.getD());
        Preferences.getDouble("Shintake Pivot FeedForward kS", kS);
        Preferences.getDouble("Shintake Pivot FeedForward kG", kG);
        Preferences.getDouble("Shintake Pivot FeedForward kV", kV);
        SmartDashboard.putNumber("STP_Angle", this.getSTPAngle());
        SmartDashboard.putNumber("STP_NEO_Encoder", this.getMotorRotations());
        SmartDashboard.putNumber("STP_MOTOR_MASTER_Rotations", this.getMotorRotations());
        SmartDashboard.putNumber("STP_Absolute_Encoder_Relative", this.absoluteEncoder.get());
        SmartDashboard.putNumber("STP_Absolute_Encoder_Absolute", this.absoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("STP_Angular_Velocity", this.getAngularVelocity().doubleValue());
        SmartDashboard.putNumber("STP_Target_Angle", this.getTargetAngle());

        // SmartDashboard.putNumber("STP_Move_P_Gain", this.movePIDController.getPTerm());
        // SmartDashboard.putNumber("STP_Move_I_Gain", this.movePIDController.getITerm());
        // SmartDashboard.putNumber("STP_Move_D_Gain", this.movePIDController.getDTerm());

        // SmartDashboard.putNumber("STP_Angle", this.getSTPAngle());
        // SmartDashboard.putNumber("STP_NEO_Encoder", this.getMotorRotations());
        // SmartDashboard.putNumber("STP_MOTOR_MASTER_Rotations", this.getMotorRotations());
        // // SmartDashboard.putNumber("STP_Cache_Offset",
        // // this.getCacheOffset());
        // // SmartDashboard.putNumber("STP_Manual_Offset",
        // // this.manualOffset.getAsDouble());
        // //SmartDashboard.putNumber("STP_Offset", this.getSTPOffset());
        // SmartDashboard.putNumber("STP_Absolute_Encoder_Relative", this.absoluteEncoder.get());
        // SmartDashboard.putNumber("STP_Absolute_Encoder_Absolute", this.absoluteEncoder.getAbsolutePosition());

        // // movePIDController.kP = SmartDashboard.getNumber("STPMoveKp", movePIDController.kP);
        // // movePIDController.kI = SmartDashboard.getNumber("STPMoveKi", movePIDController.kI);
        // // movePIDController.kD = SmartDashboard.getNumber("STPMoveKd", movePIDController.kD);

        // SmartDashboard.putNumber("STP_Angular_Velocity", this.getAngularVelocity().doubleValue());
    }
    
    public void resetSTPTargetAngle() { 
        this.motionProfileStartState = new TrapezoidProfile.State(this.getSTPAngle(), 0.0); //this.getSTPAngle(), 0.0);
        this.motionProfileEndState = new TrapezoidProfile.State(this.getSTPAngle(), 0.0);

        this.targetAngle = this.getSTPAngle(); 
    }
    

}

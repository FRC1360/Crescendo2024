package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.OrbitPID;
import frc.robot.util.OrbitTimer;

public class ArmChassisPivotSubsystem extends SubsystemBase {

    private CANSparkMax ACPMotorMaster;
    private CANSparkMax ACPMotorSlave;
    private double targetAngle;

    // we can use SparkMAX integrated PID, its got more features and is easier to
    // use
    public OrbitPID movePIDController; // PID Controller for following Trapazoid Motion Profile
    // you shouldn't need separate motion profiles for each direction, PIDF will
    // handle gravity and whatnot
    // public TrapezoidProfile.Constraints ACPUpMotionProfileConstraints;
    // public TrapezoidProfile.Constraints ACPDownMotionProfileConstraints;

    private TrapezoidProfile acpMotionProfile;
    private TrapezoidProfile.State motionProfileStartState; 
    private TrapezoidProfile.State motionProfileEndState; 

    private OrbitTimer timer; 

    public TrapezoidProfile.Constraints ACPMotionProfileConstraints; 

    private double angularVelocity; // angular velocity in deg / second
    private double lastAngle;
    private long lastTime;

    private DutyCycleEncoder absoluteEncoder;

    public ArmFeedforward ACPFeedForward;

    private double absoluteEncoderOffset = Constants.ACPConstants.ACP_ENCODER_OFFSET;

    public ArmChassisPivotSubsystem() {
        // this.holdPIDController = new OrbitPID(0.035, 0.0000075, 0.0); //kP = 0.045
        this.movePIDController = new OrbitPID(0.14, 0.0, 0.0); // kP = 0.02

        SmartDashboard.putNumber("ACPMoveKp", movePIDController.kP);
        SmartDashboard.putNumber("ACPMoveKi", movePIDController.kI);
        SmartDashboard.putNumber("ACPMoveKd", movePIDController.kD);

        // This units are deg / second for velocity and deg / sec^2 for acceleration
        // this.ACPUpMotionProfileConstraints = new TrapezoidProfile.Constraints(200.0, 350.0);
        // this.ACPDownMotionProfileConstraints = new TrapezoidProfile.Constraints(100.0, 250.0);
        this.ACPMotionProfileConstraints = new TrapezoidProfile.Constraints(200.0, 225.0); 
        this.acpMotionProfile = new TrapezoidProfile(this.ACPMotionProfileConstraints); 

        // Initalize start and end states so the robot begins at target during startup
        this.motionProfileStartState = new TrapezoidProfile.State(Constants.HOME_POSITION_ACP, 0.0); 
        this.motionProfileEndState = new TrapezoidProfile.State(Constants.HOME_POSITION_ACP, 0.0); 

        this.targetAngle = Constants.HOME_POSITION_ACP;

        this.ACPMotorMaster = new CANSparkMax(Constants.ACPConstants.ACP_MOTOR_MASTER, MotorType.kBrushless);
        this.ACPMotorSlave = new CANSparkMax(Constants.ACPConstants.ACP_MOTOR_SLAVE, MotorType.kBrushless);

        this.ACPFeedForward = new ArmFeedforward(0.0, 0.0005, 0.0); // kG = 0.001
        SmartDashboard.putNumber("ACPMoveKg", ACPFeedForward.kg);

        this.ACPMotorMaster.restoreFactoryDefaults();
        this.ACPMotorSlave.restoreFactoryDefaults();

        this.ACPMotorMaster.setIdleMode(IdleMode.kBrake);
        this.ACPMotorSlave.setIdleMode(IdleMode.kBrake);

        this.ACPMotorMaster.setSmartCurrentLimit(80);
        this.ACPMotorSlave.setSmartCurrentLimit(80);

        this.ACPMotorSlave.follow(this.ACPMotorMaster);

        this.ACPMotorMaster.setInverted(false);
        this.ACPMotorSlave.setInverted(false);

        this.ACPMotorMaster.getEncoder().setPositionConversionFactor(Constants.ACPConstants.ACP_GEAR_RATIO);

        this.absoluteEncoder = new DutyCycleEncoder(Constants.ACPConstants.ACP_ENCODER_CHANNEL);
        this.timer = new OrbitTimer(); 

        Preferences.initDouble("ACP_Target_Angle", this.getTargetAngle());
        Preferences.initDouble("ACP_Angle", this.getACPAngle());
        Preferences.initDouble("ACP_Output_Master", this.ACPMotorMaster.get());
        Preferences.initDouble("ACP_Output_Slave", this.ACPMotorSlave.get());
        Preferences.initDouble("ACP_Angular_Velocity", this.getAngularVelocity());
        Preferences.initDouble("ACP_Move_P_Gain", this.movePIDController.getPTerm());
        Preferences.initDouble("ACP_Move_I_Gain", this.movePIDController.getITerm());
        Preferences.initDouble("ACP_Move_D_Gain", this.movePIDController.getDTerm());
        Preferences.initDouble("ACP_Absolute_Encoder_Get", this.absoluteEncoder.get());
        Preferences.initDouble("ACP_Absolute_Encoder_Absolute", this.absoluteEncoder.getAbsolutePosition());
        Preferences.initDouble("ACP_Motor_Encoder", this.ACPMotorMaster.getEncoder().getPosition());
        Preferences.initDouble("ACP_Master_Current", this.ACPMotorMaster.getOutputCurrent());

        resetMotorRotations();
    }

    public double getMotorRotations() {
        return this.ACPMotorMaster.getEncoder().getPosition();
    }

    public double getACPAngle() {
        return this.rotationsToAngleConversion(this.getMotorRotations());
    }

    public void setACPSpeed(double speed) {
        /*
         * if (this.getACPAngle() > Constants.ACPConstants.MAX_ACP_ANGLE
         * || this.getACPAngle() < Constants.ACPConstants.MIN_ACP_ANGLE)
         * speed = 0.0;
         */
        this.ACPMotorMaster.set(speed);
    }

    public void resetMotorRotations() {
        //
        double newPos = -(absoluteEncoder.getAbsolutePosition() - this.absoluteEncoderOffset);

        SmartDashboard.putNumber("New_Pos", newPos);

        if (this.ACPMotorMaster.getEncoder().setPosition(newPos) == REVLibError.kOk) {
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
        // if (this.getACPAngle() > Constants.ACPConstants.MAX_ACP_ANGLE
        // || this.getACPAngle() < Constants.ACPConstants.MIN_ACP_ANGLE)
        // voltage = 0.0;

        this.ACPMotorMaster.setVoltage(voltage);
        this.ACPMotorSlave.setVoltage(voltage);
    }

    /*
     * Sets arm voltage based off 0.0 - 1.0
     */
    public void setACPNormalizedVoltage(double voltage) {
        this.setACPVoltage(voltage * 12.0); // Should probably change this to a constant somewhere for ARM_VOLTAGE
    }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;

        this.movePIDController.reset();

        this.motionProfileStartState = new TrapezoidProfile.State(this.getACPAngle(), this.getAngularVelocity());
        this.motionProfileEndState = new TrapezoidProfile.State(targetAngle, 0.0); 
        this.timer.start(); 

        System.out.println("Target angle for ACP scheduled for: " + targetAngle); 
    }

    public double getTargetAngle() {
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

    private void updateAngularVelocity() {
        long currentTime = System.currentTimeMillis();

        double deltaTime = (currentTime - lastTime) / 1000.0;

        this.angularVelocity = (this.getACPAngle() - lastAngle) / deltaTime;
        this.lastAngle = this.getACPAngle();
        this.lastTime = currentTime;
    }

    public double getAngularVelocity() {
        return this.angularVelocity;
    }

    public boolean atTarget() {
        return Math.abs(this.getTargetAngle() - this.getACPAngle()) <= 3.0 
                    || this.acpMotionProfile.isFinished(this.timer.getTimeDeltaSec()); // Should make this a constant
    }

    // this should probably be a flag set by commands when they start moving the
    // arm, not something that you check this way
    // this isnt very flexible and may give bad data near the target positions
    // public void checkTransitioning() {
    //     transitioning = !(Math.abs(this.getACPAngle()) < 2)
    //             && (this.getScheduledAngle() > 0.0 && this.getACPAngle() < 0.0)
    //             || (this.getScheduledAngle() < 0.0 && this.getACPAngle() > 0.0);
    // }

    public void resetEncoderOffset() {
        this.absoluteEncoderOffset = this.absoluteEncoder.getAbsolutePosition();
    }

    public double calculateControlLoopOutput() { 
        TrapezoidProfile.State profileTarget = this.acpMotionProfile.calculate(this.timer.getTimeDeltaSec(), this.motionProfileEndState,
                this.motionProfileStartState);

        double target = profileTarget.position; 
        double input = this.getACPAngle(); 

        double pidOut = this.movePIDController.calculate(target, input); 

        double feedforwardOutput = this.ACPFeedForward.calculate(
            Math.toRadians(profileTarget.position),
            Math.toRadians(this.getAngularVelocity()));

        return pidOut + feedforwardOutput; 
    }

    @Override
    public void periodic() {
        updateAngularVelocity();
        updateSmartDashboard();

        // All of Control Loop motion is done within the subsystem -- simply set a target angle and the subsystem will go there
        if (!this.acpMotionProfile.isFinished(this.timer.getTimeDeltaSec())) {
            double out = calculateControlLoopOutput(); 
            SmartDashboard.putNumber("ACP_Control_Loop_Out", out); 
            this.setACPNormalizedVoltage(out);
        }
        // SmartDashboard.putNumber("Current Angle: ", this.getACPAngle());
        // SmartDashboard.putNumber("Target Angle: ", true);
    }

    public void updateSmartDashboard() {

        Preferences.getDouble("ACP_Target_Angle", this.getTargetAngle());
        Preferences.getDouble("ACP_Angle", this.getACPAngle());
        Preferences.getDouble("ACP_Output_Master", this.ACPMotorMaster.get());
        Preferences.getDouble("ACP_Output_Slave", this.ACPMotorSlave.get());
        Preferences.getDouble("ACP_Angular_Velocity", this.getAngularVelocity());
        Preferences.getDouble("ACP_Move_P_Gain", this.movePIDController.getPTerm());
        Preferences.getDouble("ACP_Move_I_Gain", this.movePIDController.getITerm());
        Preferences.getDouble("ACP_Move_D_Gain", this.movePIDController.getDTerm());
        Preferences.getDouble("ACP_Absolute_Encoder_Get", this.absoluteEncoder.get());
        Preferences.getDouble("ACP_Absolute_Encoder_Absolute", this.absoluteEncoder.getAbsolutePosition());
        Preferences.getDouble("ACP_Motor_Encoder", this.ACPMotorMaster.getEncoder().getPosition());
        Preferences.getDouble("ACP_Master_Current", this.ACPMotorMaster.getOutputCurrent());

        // SmartDashboard.putNumber("ACP_Target_Angle", this.getTargetAngle());
        // SmartDashboard.putNumber("ACP_Angle", this.getACPAngle());

        // SmartDashboard.putNumber("ACP_Output_Master", this.ACPMotorMaster.get());
        // SmartDashboard.putNumber("ACP_Output_Slave", this.ACPMotorSlave.get());

        // SmartDashboard.putNumber("ACP_Angular_Velocity", this.getAngularVelocity());

        // SmartDashboard.putNumber("ACP_Move_P_Gain", this.movePIDController.getPTerm());
        // SmartDashboard.putNumber("ACP_Move_I_Gain", this.movePIDController.getITerm());
        // SmartDashboard.putNumber("ACP_Move_D_Gain", this.movePIDController.getDTerm());

        // // SmartDashboard.putBoolean("ACP_Transition_State", transitioning);

        // SmartDashboard.putNumber("ACP_Absolute_Encoder_Get", this.absoluteEncoder.get());
        // SmartDashboard.putNumber("ACP_Absolute_Encoder_Absolute", this.absoluteEncoder.getAbsolutePosition());
        // SmartDashboard.putNumber("ACP_Motor_Encoder", this.ACPMotorMaster.getEncoder().getPosition());

        // SmartDashboard.putNumber("ACP_Master_Current", this.ACPMotorMaster.getOutputCurrent());
        /*
         * movePIDController.kP = SmartDashboard.getNumber("ACPMoveKp",
         * movePIDController.kP);
         * movePIDController.kI = SmartDashboard.getNumber("ACPMoveKi",
         * movePIDController.kI);
         * movePIDController.kD = SmartDashboard.getNumber("ACPMoveKd",
         * movePIDController.kD);
         */
        //ACPFeedForward = new ArmFeedforward(0, SmartDashboard.getNumber("ACPMoveKg", ACPFeedForward.kg), 0);
    }
}
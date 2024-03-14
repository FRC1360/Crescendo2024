package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

public class ArmChassisPivotSubsystem extends SubsystemBase {

    private CANSparkMax ACPMotorMaster;
    private CANSparkMax ACPMotorSlave;
    private double targetAngle;

    // we can use SparkMAX integrated PID, its got more features and is easier to
    // use
    public PIDController movePIDController; // PID Controller for following Trapazoid Motion Profile
    // you shouldn't need separate motion profiles for each direction, PIDF will
    // handle gravity and whatnot
    // public TrapezoidProfile.Constraints ACPUpMotionProfileConstraints;
    // public TrapezoidProfile.Constraints ACPDownMotionProfileConstraints;

    private TrapezoidProfile acpMotionProfile;
    private TrapezoidProfile.State motionProfileStartState;
    private TrapezoidProfile.State motionProfileEndState;

    private OrbitTimer timer;

    public TrapezoidProfile.Constraints ACPMotionProfileConstraints;

    private TrapezoidProfile slowProfile;
    private TrapezoidProfile.State slowProfileStartState;
    private TrapezoidProfile.State slowProfileEndState;

    private double angularVelocity; // angular velocity in deg / second
    private double lastAngle;
    private double lastTime = -1;

    private DutyCycleEncoder absoluteEncoder;

    public ArmFeedforward ACPFeedForward;

    private double absoluteEncoderOffset = Constants.ACPConstants.ACP_ENCODER_OFFSET;

    private double cacheOffset = 0.0;
    private boolean goingUp = true;
    private boolean specialCase = false;

    private double kP = 0.0325;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kS = 0.0;
    private double kG = 0.0;
    private double kV = 0.0;

    private double slowMaxVelocity;
    private boolean slowInit = false;
    private OrbitTimer slowTimer;
    private double timeElapsed;

    private double maxVelocity;

    public InterpolatingDoubleTreeMap armFarShotMap;

    public ArmChassisPivotSubsystem() {
        // this.holdPIDController = new OrbitPID(0.035, 0.0000075, 0.0); //kP = 0.045
        // this.movePIDController = new PIDController(0.0325, 0.0, 0.0); // kP = 0.02
        this.movePIDController = new PIDController(kP, kI, kD);

        this.ACPMotorMaster = new CANSparkMax(Constants.ACPConstants.ACP_MOTOR_MASTER, MotorType.kBrushless);
        this.ACPMotorSlave = new CANSparkMax(Constants.ACPConstants.ACP_MOTOR_SLAVE, MotorType.kBrushless);

        // this.ACPFeedForward = new ArmFeedforward(0.0, 0.0, 0.0);
        this.ACPFeedForward = new ArmFeedforward(kS, kG, kV);
        SmartDashboard.putNumber("ACPMoveKg", ACPFeedForward.kg);

        this.ACPMotorMaster.restoreFactoryDefaults();
        this.ACPMotorSlave.restoreFactoryDefaults();

        this.ACPMotorMaster.setIdleMode(IdleMode.kBrake);
        this.ACPMotorSlave.setIdleMode(IdleMode.kBrake);

        this.ACPMotorMaster.setSmartCurrentLimit(80);
        this.ACPMotorSlave.setSmartCurrentLimit(80);

        this.ACPMotorSlave.follow(this.ACPMotorMaster);

        this.ACPMotorMaster.setInverted(true);
        // this.ACPMotorSlave.setInverted(true);

        this.ACPMotorMaster.getEncoder().setPositionConversionFactor(Constants.ACPConstants.ACP_GEAR_RATIO);

        this.absoluteEncoder = new DutyCycleEncoder(Constants.ACPConstants.ACP_ENCODER_CHANNEL);

        this.maxVelocity = 60.0;
        // This units are deg / second for velocity and deg / sec^2 for acceleration
        this.ACPMotionProfileConstraints = new TrapezoidProfile.Constraints(this.maxVelocity, 60);
        this.acpMotionProfile = new TrapezoidProfile(this.ACPMotionProfileConstraints);

        this.slowMaxVelocity = 25.0;
        this.slowProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(this.slowMaxVelocity, 15.0));

        Preferences.initDouble("ACP_Move_P_Gain", this.movePIDController.getP());
        Preferences.initDouble("ACP_Move_I_Gain", this.movePIDController.getI());
        Preferences.initDouble("ACP_Move_D_Gain", this.movePIDController.getD());
        Preferences.initDouble("ACP kP", kP);
        Preferences.initDouble("ACP kI", kI);
        Preferences.initDouble("ACP kD", kD);
        Preferences.initDouble("ACP FeedForward kS", kS);
        Preferences.initDouble("ACP FeedForward kG", kG);
        Preferences.initDouble("ACP FeedForward kV", kV);

        resetMotorRotations();

        // Initalize start and end states so the robot goes to target during startup
        this.timer = new OrbitTimer();
        this.motionProfileStartState = new TrapezoidProfile.State(this.getACPAngle(), 0.0); // this.getACPAngle(), 0.0);
        this.motionProfileEndState = new TrapezoidProfile.State(this.getACPAngle(), 0.0);

        this.slowProfileStartState = new TrapezoidProfile.State(70.0, 0.0);
        this.slowProfileEndState = new TrapezoidProfile.State(90.0, 0.0);

        this.targetAngle = this.getACPAngle(); // Constants.HOME_POSITION_ACP;
        this.slowTimer = new OrbitTimer();

        this.armFarShotMap = new InterpolatingDoubleTreeMap();

        this.armFarShotMap.put(0.5, 76.2);
        this.armFarShotMap.put(0.75, 57.6);
        this.armFarShotMap.put(1.00, 48.1);
        this.armFarShotMap.put(1.25, 41.8);
        this.armFarShotMap.put(1.5, 37.3);
        this.armFarShotMap.put(1.75, 33.8);
        this.armFarShotMap.put(2.0, 30.9);
        this.armFarShotMap.put(2.25, 28.5);
        this.armFarShotMap.put(2.50, 26.5);
        this.armFarShotMap.put(2.75, 24.7);
        this.armFarShotMap.put(3.00, 23.2);
    }

    public double getMotorRotations() {
        return this.ACPMotorMaster.getEncoder().getPosition();
    }

    public double getACPAngle() {
        return this.rotationsToAngleConversion(this.getMotorRotations()) + Constants.HOME_POSITION_ACP;
    }

    public void setACPSpeed(double speed) {
        /*
         * if (this.getACPAngle() > Constants.ACPConstants.MAX_ACP_ANGLE
         * || this.getACPAngle() < Constants.ACPConstants.MIN_ACP_ANGLE)
         * speed = 0.0;
         */
        this.ACPMotorMaster.set(speed);
    }

    public void setCacheOffset(double offset) {
        this.cacheOffset += offset;
    }

    public void resetMotorRotations() {
        if (this.absoluteEncoder.getAbsolutePosition() == 0.0) {
            DriverStation.reportError("ACP Absolute encoder reports 0.0! Possibly not connected properly!", true);
            System.exit(1);
        }

        double newPos = (absoluteEncoder.getAbsolutePosition() - this.absoluteEncoderOffset);

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
        if (this.getACPAngle() > Constants.ACPConstants.MAX_ACP_ANGLE
                || this.getACPAngle() < Constants.ACPConstants.MIN_ACP_ANGLE) {
            voltage = 0.0;
        } else if (this.getAngularVelocity() > this.maxVelocity + 15.0) {
            DriverStation.reportError("Tried to send ACP faster than " + this.maxVelocity + 15.0 + "; Actual velocity: "
                    + this.getAngularVelocity(), true);
            System.exit(1);
        }

        this.ACPMotorMaster.setVoltage(voltage);
        // this.ACPMotorSlave.setVoltage(voltage);
    }

    /*
     * Sets arm voltage based off 0.0 - 1.0
     */
    public void setACPNormalizedVoltage(double voltage) {
        this.setACPVoltage(voltage * 12.0); // Should probably change this to a constant somewhere for ARM_VOLTAGE
    }

    public void setTargetAngle(double targetAngle) {
        if (this.targetAngle == targetAngle) {
            return;
        } else if (this.targetAngle >= Constants.ACPConstants.MAX_ACP_ANGLE
                || this.targetAngle <= Constants.ACPConstants.MIN_ACP_ANGLE) {
            DriverStation.reportError("Tried to set ACP to above or below max or min; Target: " + this.targetAngle,
                    true);
            System.exit(1);
        } else if (this.getACPAngle() < Constants.ACPConstants.ACP_SLOP_OCCUR_ANGLE
                && targetAngle >= Constants.ACPConstants.ACP_SLOP_OCCUR_ANGLE) {
            this.motionProfileStartState = new TrapezoidProfile.State(this.getACPAngle(), this.getAngularVelocity());
            this.motionProfileEndState = new TrapezoidProfile.State(Constants.ACPConstants.ACP_SLOP_OCCUR_ANGLE,
                    this.slowMaxVelocity);

            this.slowProfileStartState = new TrapezoidProfile.State(Constants.ACPConstants.ACP_SLOP_OCCUR_ANGLE,
                    this.getAngularVelocity());
            this.slowProfileEndState = new TrapezoidProfile.State(targetAngle, 0.0);
            slowInit = false;
            goingUp = true;
            specialCase = true;
        } else if (this.getACPAngle() > Constants.ACPConstants.ACP_SLOP_OCCUR_ANGLE
                && targetAngle <= Constants.ACPConstants.ACP_SLOP_OCCUR_ANGLE) {
            this.slowProfileStartState = new TrapezoidProfile.State(this.getACPAngle(), this.getAngularVelocity());
            this.slowProfileEndState = new TrapezoidProfile.State(Constants.ACPConstants.ACP_SLOP_OCCUR_ANGLE,
                    this.slowMaxVelocity);

            this.motionProfileStartState = new TrapezoidProfile.State(Constants.ACPConstants.ACP_SLOP_OCCUR_ANGLE,
                    this.getAngularVelocity());
            this.motionProfileEndState = new TrapezoidProfile.State(targetAngle, 0.0);
            slowInit = false;
            goingUp = false;
            specialCase = true;
        } else {
            this.motionProfileStartState = new TrapezoidProfile.State(this.getACPAngle(), this.getAngularVelocity());
            this.motionProfileEndState = new TrapezoidProfile.State(targetAngle, 0.0);
            specialCase = false;
        }
        this.targetAngle = targetAngle + this.cacheOffset;

        this.movePIDController.reset();
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
        double currentTime = (System.currentTimeMillis() / 1000.0);

        if (lastTime != -1) {
            double deltaTime = (currentTime - lastTime); /// 1000.0;

            this.angularVelocity = (this.getACPAngle() - lastAngle) / deltaTime;
        }
        this.lastAngle = this.getACPAngle();
        this.lastTime = currentTime;
    }

    public double getAngularVelocity() {
        return this.angularVelocity;
    }

    public boolean atTarget() {
        return Math.abs(this.getTargetAngle() - this.getACPAngle()) <= Constants.ACPConstants.ACP_GO_TO_POS_TOLERANCE; // Should
                                                                                                                       // make
                                                                                                                       // this
                                                                                                                       // a
                                                                                                                       // constant
    }

    // this should probably be a flag set by commands when they start moving the
    // arm, not something that you check this way
    // this isnt very flexible and may give bad data near the target positions
    // public void checkTransitioning() {
    // transitioning = !(Math.abs(this.getACPAngle()) < 2)
    // && (this.getScheduledAngle() > 0.0 && this.getACPAngle() < 0.0)
    // || (this.getScheduledAngle() < 0.0 && this.getACPAngle() > 0.0);
    // }

    public void resetEncoderOffset() {
        this.absoluteEncoderOffset = this.absoluteEncoder.getAbsolutePosition();
    }

    private double calculateControlLoopOutput() {
        TrapezoidProfile.State profileTarget = this.acpMotionProfile.calculate(this.timer.getTimeDeltaSec(),
                this.motionProfileStartState,
                this.motionProfileEndState);
        if (specialCase) {
            if (goingUp) {
                if (this.acpMotionProfile.isFinished(this.timer.getTimeDeltaSec()) && !slowInit) {
                    timeElapsed = this.timer.getTimeDeltaSec();
                    slowInit = true;
                }
                if (slowInit) {
                    profileTarget = this.slowProfile.calculate(this.timer.getTimeDeltaSec() - timeElapsed,
                            this.slowProfileStartState, this.slowProfileEndState);
                }
            } else {
                profileTarget = this.slowProfile.calculate(this.timer.getTimeDeltaSec(), this.slowProfileStartState,
                        this.slowProfileEndState);

                if (!slowInit && this.slowProfile.isFinished(this.timer.getTimeDeltaSec())) {
                    timeElapsed = this.timer.getTimeDeltaSec();
                    slowInit = true;
                }
                if (slowInit) {
                    profileTarget = this.acpMotionProfile.calculate(this.timer.getTimeDeltaSec() - timeElapsed,
                            this.motionProfileStartState, this.motionProfileEndState);
                }
            }
        }

        double target = profileTarget.position;
        double input = this.getACPAngle();

        SmartDashboard.putNumber("ACP_Profile_Position", target);
        SmartDashboard.putNumber("ACP_Profile_Velocity", profileTarget.velocity);

        double pidOut = this.movePIDController.calculate(input, target);

        double feedforwardOutput = this.ACPFeedForward.calculate(
                Math.toRadians(profileTarget.position),
                Math.toRadians(this.getAngularVelocity()));

        return pidOut;
    }

    @Override
    public void periodic() {
        updateAngularVelocity();
        updateSmartDashboard();
        // this.movePIDController = new PIDController(kP, kI, kD); // added this here to
        // make sure it updates on the fly

        // All of Control Loop motion is done within the subsystem -- simply set a
        // target angle and the subsystem will go there
        // When the motion profile is finished, the result which it outputs will be the
        // goal, making it a PID/FF control loop only
        double out = calculateControlLoopOutput();
        SmartDashboard.putNumber("ACP_Control_Loop_Out", out);
        this.setACPNormalizedVoltage(out);
        // SmartDashboard.putNumber("Current Angle: ", this.getACPAngle());
        // SmartDashboard.putNumber("Target Angle: ", true);
    }

    public void updateSmartDashboard() {

        Preferences.getDouble("ACP_Move_P_Gain", this.movePIDController.getP());
        Preferences.getDouble("ACP_Move_I_Gain", this.movePIDController.getI());
        Preferences.getDouble("ACP_Move_D_Gain", this.movePIDController.getD());
        Preferences.getDouble("ACP_Absolute_Encoder_Get", this.absoluteEncoder.get());
        Preferences.getDouble("ACP_Absolute_Encoder_Absolute", this.absoluteEncoder.getAbsolutePosition());
        Preferences.getDouble("ACP_Motor_Encoder", this.ACPMotorMaster.getEncoder().getPosition());

        Preferences.getDouble("ACP kP", kP);
        Preferences.getDouble("ACP kI", kI);
        Preferences.getDouble("ACP kD", kD);
        Preferences.getDouble("ACP FeedForward kG", kG);
        Preferences.getDouble("ACP FeedForward kV", kV);
        SmartDashboard.putNumber("ACP_Target_Angle", this.getTargetAngle());
        SmartDashboard.putNumber("ACP_Angle", this.getACPAngle());
        SmartDashboard.putNumber("ACP_Output_Master", this.ACPMotorMaster.get());
        SmartDashboard.putNumber("ACP_Angular_Velocity", this.getAngularVelocity());
        SmartDashboard.putNumber("ACP_Absolute_Encoder_Get", this.absoluteEncoder.get());
        SmartDashboard.putNumber("ACP_Absolute_Encoder_Absolute", this.absoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("ACP_Motor_Encoder", this.ACPMotorMaster.getEncoder().getPosition());
        SmartDashboard.putNumber("ACP_Master_Current", this.ACPMotorMaster.getOutputCurrent());

        // SmartDashboard.putNumber("ACP_Move_P_Gain",
        // this.movePIDController.getPTerm());
        // SmartDashboard.putNumber("ACP_Move_I_Gain",
        // this.movePIDController.getITerm());
        // SmartDashboard.putNumber("ACP_Move_D_Gain",
        // this.movePIDController.getDTerm());

        // SmartDashboard.putBoolean("ACP_Transition_State", transitioning);

        /*
         * movePIDController.kP = SmartDashboard.getNumber("ACPMoveKp",
         * movePIDController.kP);
         * movePIDController.kI = SmartDashboard.getNumber("ACPMoveKi",
         * movePIDController.kI);
         * movePIDController.kD = SmartDashboard.getNumber("ACPMoveKd",
         * movePIDController.kD);
         */
        // ACPFeedForward = new ArmFeedforward(0, SmartDashboard.getNumber("ACPMoveKg",
        // ACPFeedForward.kg), 0);
    }

    public void resetArmTargetAngle() {
        this.motionProfileStartState = new TrapezoidProfile.State(this.getACPAngle(), 0.0); // this.getACPAngle(), 0.0);
        this.motionProfileEndState = new TrapezoidProfile.State(this.getACPAngle(), 0.0);

        this.targetAngle = this.getACPAngle(); // Constants.HOME_POSITION_ACP;
    }
}
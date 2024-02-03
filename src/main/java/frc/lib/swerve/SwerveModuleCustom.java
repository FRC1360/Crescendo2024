package frc.lib.swerve;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.MagEncoder;
import frc.lib.util.PIDConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import swervelib.encoders.CANCoderSwerve;
import swervelib.math.SwerveModuleState2;

public class SwerveModuleCustom {
    /* Module details */
    public int moduleNumber;

    /* Motors */
    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    /* Encoders and their values */
    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    //public MagEncoder angleEncoder;
    public CANCoderSwerve angleEncoder; 
    private double lastAngle;
    private double angleOffset;

    /* Controllers */
    public final SparkPIDController driveController;
    public final SparkPIDController angleController;
    public final PIDConstants anglePID;
    public final SimpleMotorFeedforward driveSVA;
    public SimpleMotorFeedforward feedforward;

    @AutoLogOutput(key = "Swerve/Modules/M{moduleNumber}/TargetSpeed")
    public double targetSpeed = 0;

    @AutoLogOutput(key = "Swerve/Modules/M{moduleNumber}/TargetAngle")
    public double targetAngle = 0;

    public SwerveModuleCustom(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;

        angleOffset = moduleConstants.angleOffset;
        this.anglePID = moduleConstants.anglePID;
        this.driveSVA = moduleConstants.driveSVA;

        /* Angle Encoder Config */
        angleEncoder = new CANCoderSwerve(moduleConstants.canCoderID);  
        
        System.out.println(angleEncoder.setAbsoluteEncoderOffset(moduleConstants.angleOffset));

        /* Angle Motor Config */
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();
    }

    public void updateDashboardValues() {
        SmartDashboard.putNumber(Constants.Swerve.MODULE_NAMES[this.moduleNumber] + " Integrated Encoder",
                this.getState().angle.getDegrees());
        SmartDashboard.putNumber(Constants.Swerve.MODULE_NAMES[this.moduleNumber] + " Mag Encoder",
                this.getCanCoder().getDegrees());
        SmartDashboard.putNumber(Constants.Swerve.MODULE_NAMES[this.moduleNumber] + " Set Point", this.lastAngle);
        SmartDashboard.putNumber(Constants.Swerve.MODULE_NAMES[this.moduleNumber] + " Drive Encoder Velocity",
                this.driveEncoder.getVelocity());
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // this.updateControllerValues();
        desiredState = OnboardModuleState.optimize(
                desiredState,
                getState().angle); // Custom optimize command, since default WPILib optimize assumes
        // continuous controller which REV and CTRE are not

        this.setSpeed(desiredState, isOpenLoop);
        this.setAngle(desiredState);
    }

    public void setDesiredState(SwerveModuleState2 desiredState, boolean isOpenLoop) {
        this.setSpeed(desiredState, isOpenLoop);
        this.setAngle(desiredState);
    }

    public void resetToAbsolute() {
        integratedAngleEncoder.setPosition(this.getCanCoder().getDegrees());
    }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        angleMotor.setSmartCurrentLimit(Constants.Swerve.CONTINUOUS_CURRENT_LIMIT);
        angleMotor.setInverted(Constants.Swerve.ANGLE_INVERT);
        angleMotor.setIdleMode(Constants.Swerve.IDLE_MODE);
        integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.ANGLE_CONVERSION_FACTOR);
        this.anglePID.applyPID(this.angleController);
        angleController.setFF(0);
        angleMotor.enableVoltageCompensation(12.0);
        this.resetToAbsolute();
    }

    private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        driveMotor.setSmartCurrentLimit(Constants.Swerve.PEAK_CURRENT_LIMIT, Constants.Swerve.CONTINUOUS_CURRENT_LIMIT);
        driveMotor.setInverted(Constants.Swerve.DRIVE_INVERT);
        driveMotor.setIdleMode(Constants.Swerve.IDLE_MODE);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.DRIVE_CONVERSION_VELOCITY_FACTOR);
        driveEncoder.setPositionConversionFactor(Constants.Swerve.DRIVE_CONVERSION_POSITION_FACTOR);
        Constants.Swerve.drivePID.applyPID(this.driveController);
        driveController.setFF(0);
        this.feedforward = driveSVA;
        driveMotor.enableVoltageCompensation(12.0);
        driveEncoder.setPosition(0.0);

        // if (/* this.moduleNumber == 0 || */ this.moduleNumber == 2) {  // Noticed that drive motor at swerve 0 is not behaving properly
        //     this.driveMotor.setInverted(!Constants.Swerve.DRIVE_INVERT);
        // }
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.ROBOT_MAX_VELOCITY_METERS_PER_SECOND;
            driveMotor.set(percentOutput);
        } else {
            this.targetSpeed = desiredState.speedMetersPerSecond;
            driveController.setReference(
                    desiredState.speedMetersPerSecond,
                    ControlType.kVelocity,
                    0,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
            // SmartDashboard.putNumber(Constants.Swerve.MODULE_NAMES[this.moduleNumber] + "
            // Drive Set Velocity",
            // desiredState.speedMetersPerSecond);
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.MAX_SPEED * 0.01))
                ? lastAngle
                : desiredState.angle
                        .getDegrees();

        angleController.setReference(angle, ControlType.kPosition);
        lastAngle = angle;
        this.targetAngle = angle;
    }

    public void goToHome() {
        Rotation2d angle = getAngle();
        angleController.setReference(angle.getDegrees() - angle.getDegrees() % 360,
                ControlType.kPosition);
        lastAngle = angle.getDegrees() - angle.getDegrees() % 360;
    }

    @AutoLogOutput(key = "Swerve/Modules/M{moduleNumber}/AbsAngle")
    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(this.angleEncoder.getAbsolutePosition());
    }

    @AutoLogOutput(key = "Swerve/Modules/M{moduleNumber}/Angle")
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(this.integratedAngleEncoder.getPosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(this.getSpeed(), this.getAngle());
    }

    @AutoLogOutput(key = "Swerve/Modules/M{moduleNumber}/Speed")
    public double getSpeed() {
        return this.driveEncoder.getVelocity();
    }

    public double getDistance() {
        return this.driveEncoder.getPosition();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.getDistance(), this.getAngle());
    }

    public SwerveModulePosition getRedPosition() {
        return new SwerveModulePosition(this.getDistance(), Rotation2d.fromDegrees(-this.getAngle().getDegrees()));
    }

    public CANSparkMax getDriveMotor() {
        return this.driveMotor;
    }
}
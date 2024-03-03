package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.controller.PIDController;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.SwerveModuleCustom;
import frc.lib.util.NavX;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveAutoConfig;
import frc.robot.util.PIDSwerveValues;
import swervelib.math.SwerveModuleState2;

public class SwerveSubsystem extends SubsystemBase {
	public final NavX navX;
	private SwerveModuleCustom[] swerveModules;
	private Translation2d centerOfRotation = new Translation2d();
	private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

	private Pose2d lastPose = new Pose2d(0, 0, new Rotation2d());
	private long lastPoseTimestamp = System.currentTimeMillis();

	private double XkP = 1.25; // for driveXPID || replaces Constants.Swerve.driveAlignPID.p, Constants.Swerve.driveAlignPID.i, Constants.Swerve.driveAlignPID.d
    private double XkI = 0.0;
    private double XkD = 0.0;
    private double YkP = 1.25; // for driveYPID || replaces Constants.Swerve.driveAlignPID.p, Constants.Swerve.driveAlignPID.i, Constants.Swerve.driveAlignPID.d
    private double YkI = 0.0;
    private double YkD = 0.0;
	private double AkP = 0.023; // A as in Angle for anglePID || replaces onstants.Swerve.anglePID.p, Constants.Swerve.anglePID.i, Constants.Swerve.anglePID.d
    private double AkI = 0.000001;
    private double AkD = 0.0;

	public boolean manualDrive = false;

	private PhotonCameraWrapper pCameraWrapper;

	private PIDController driveXPID = new PIDController(XkP, XkI, XkD); 
	
	private PIDController driveYPID = new PIDController(YkP, YkI, YkD); 

	private PIDController anglePID = new PIDController(AkP, AkI, AkD); 

	@AutoLogOutput(key = "Swerve/CurrentVelocity")
	public Translation2d currentVelocity = new Translation2d(0, 0);

	private Pose2d prevTargetPose = new Pose2d(); 

	public SwerveSubsystem() {
		// Gyro setup
		navX = new NavX();
		navX.setInverted(Constants.Swerve.isGyroInverted);

		pCameraWrapper = new PhotonCameraWrapper();
		// Swerve module setup
		swerveModules = new SwerveModuleCustom[] {
				new SwerveModuleCustom(0, Constants.Swerve.Mod0.constants),
				new SwerveModuleCustom(1, Constants.Swerve.Mod1.constants),
				new SwerveModuleCustom(2, Constants.Swerve.Mod2.constants),
				new SwerveModuleCustom(3, Constants.Swerve.Mod3.constants),
		};

		// Pose estimator
		swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, navX.getYaw(),
				getPositions(), new Pose2d());

		// Configure the AutoBuilder that handles all the auto path following!!
		SwerveAutoConfig.configureAutoBuilder(this);

		Preferences.initDouble("Swerve DriveX kP", this.XkP);
		Preferences.initDouble("Swerve DriveX kI", this.XkI);
		Preferences.initDouble("Swerve DriveX kD", this.XkD);
		Preferences.initDouble("Swerve DriveY kP", this.YkP);
		Preferences.initDouble("Swerve DriveY kI", this.YkI);
		Preferences.initDouble("Swerve DriveY kD", this.YkD);
		Preferences.initDouble("Swerve Angle kP", this.AkP);
		Preferences.initDouble("Swerve Angle kI", this.AkI);
		Preferences.initDouble("Swerve Angle kD", this.AkD);

	}

	public void toggleManualDrive() {
		manualDrive = !manualDrive;
	}

	public void zeroGyro() {
		this.navX.resetGyro();
	}

	public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
		SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(
								translation.getX(), translation.getY(),
								-rotation,
								manualDrive ? navX.getYaw()
										: swerveDrivePoseEstimator.getEstimatedPosition().getRotation())
						: new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
				this.centerOfRotation);
		// Get rid of tiny tiny movements in the wheels to have more consistent driving
		// experience
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

		// set the states for each module
		for (SwerveModuleCustom mod : swerveModules) {
			mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
		}
	}

	public void updateAbsAngleSmartDashboard() { 
		for (int i = 0; i < 4; i++) { 
			SmartDashboard.putNumber("Mod " + i + " Abs angle", swerveModules[i].getCanCoder().getDegrees()); 
		}
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

		for (SwerveModuleCustom mod : swerveModules) {
			mod.setDesiredState(desiredStates[mod.moduleNumber], false);
		}
	}

	public void setModuleStatesDuringAuto(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.AutoConstants.maxSpeed);

		for (SwerveModuleCustom mod : swerveModules) {
			mod.setDesiredState(desiredStates[mod.moduleNumber], false);
		}
	}

	public void setModuleStates(SwerveModuleState2[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

		for (SwerveModuleCustom mod : swerveModules) {
			mod.setDesiredState(desiredStates[mod.moduleNumber], false);
		}
	}

	public void resetModuleZeros() {
		for (SwerveModuleCustom mod : swerveModules) {
			mod.resetToAbsolute();
		}
	}

	@AutoLogOutput(key = "Swerve/ModuleStates")
	public SwerveModuleState[] getStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (SwerveModuleCustom mod : swerveModules) {
			states[mod.moduleNumber] = mod.getState();
		}
		return states;
	}

	@AutoLogOutput(key = "Swerve/DesiredModuleStates")
	public SwerveModuleState[] getDesiredStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (SwerveModuleCustom mod : swerveModules) {
			states[mod.moduleNumber] = mod.getDesiredState();
		}
		return states;
	}

	public SwerveModulePosition[] getPositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];

		for (SwerveModuleCustom mod : swerveModules) {
			positions[mod.moduleNumber] = mod.getPosition();
		}

		return positions;
	}

	public void brake() {
		System.out.println("Braking");
		SwerveModuleState2[] states = new SwerveModuleState2[4];

		for (int i = 0; i < 4; i++) {
			states[i] = new SwerveModuleState2();
			states[i].speedMetersPerSecond = 0.0;
			states[i].omegaRadPerSecond = 0.0;
		}

		states[0].angle = Rotation2d.fromDegrees(45.0);
		states[1].angle = Rotation2d.fromDegrees(-45.0);
		states[2].angle = Rotation2d.fromDegrees(-45.0);
		states[3].angle = Rotation2d.fromDegrees(45.0);

		setModuleStates(states);
	}

	public void setCoR(Translation2d centerOfRotation) {
		this.centerOfRotation = centerOfRotation;
	}

	public void resetCoR() {
		setCoR(new Translation2d());
	}

	@AutoLogOutput(key = "Swerve/CurrentPose")
	public Pose2d currentPose() {
		return swerveDrivePoseEstimator.getEstimatedPosition();
	}

	// use this to check if stuff like pathfinding is working properly
	// this has a log output so you can see if its actually calling the method
	public Pose2d currentPoseDebug() {
		System.out.print("Got pose from swerve subsystem: ");
		System.out.println(swerveDrivePoseEstimator.getEstimatedPosition());
		return swerveDrivePoseEstimator.getEstimatedPosition();
	}

	public Translation2d currentTranslation() {
		return currentPose().getTranslation();
	}

	public void setCurrentPose(Pose2d newPose) {
		swerveDrivePoseEstimator.resetPosition(
				navX.getYaw(),
				getPositions(),
				newPose);
	}

	public void resetFieldPosition() {
		setCurrentPose(new Pose2d());
	}

	public String getFormattedPose() {
		var pose = currentPose();
		return String.format("(%.2f, %.2f) %.2f degrees",
				pose.getX(),
				pose.getY(),
				pose.getRotation().getDegrees());
	}

	public void driveRobotRelative(ChassisSpeeds speeds) {
		this.drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond,
				false, false);
	}

	public ChassisSpeeds getRobotRelativeSpeeds() {
		return Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
	}

	public boolean isInRange(Pose2d target, double positionTolerance, double angleTolerance) {
		Transform2d error = target.minus(currentPose());
		return error.getX() < positionTolerance && error.getY() < positionTolerance; // &&
																						// error.getRotation().getRadians()
																						// < angleTolerance;
	}

	public double calculatePIDAngleOutput(double targetAngDeg) {
		if (targetAngDeg != prevTargetPose.getRotation().getDegrees()) { 
			this.anglePID.reset();
			this.prevTargetPose = new Pose2d(0, 0, Rotation2d.fromDegrees(targetAngDeg)); 
		} 
		System.out.println("Aligning to angle: " + targetAngDeg); 

		Pose2d currentPose = this.currentPose();

		// Constrain angles between 0 and 360

        double currentAngle = (currentPose.getRotation().getDegrees() + 360.0) % (360.0);
        double targetAngle = (targetAngDeg + 360.0) % (360.0);

        if (Math.abs(targetAngDeg) < 20) {
            // If aligning near 0, use the -180 to 180 alignment (built in Rotation2d) to
            // prevent rollover
            currentAngle = currentPose.getRotation().getDegrees();
            targetAngle = targetAngDeg;
        }

        System.out.println("Current angle: " + currentAngle + " Target Angle: " + targetAngle);

		System.out.println("Error Angle" + anglePID.getPositionError());

        // Negated because a positive rotation is perceived as going CW (for the
        // joysticks)
        // But robot angle is positive rotating CCW
        double angleOutput = -this.anglePID.calculate(currentAngle, targetAngle);

		return angleOutput; 
	}

	public boolean drivePIDAtTarget() { 
		return this.driveXPID.atSetpoint() && this.driveYPID.atSetpoint() && this.anglePID.atSetpoint(); 
	}

	public PIDSwerveValues calculatePIDDriveOutput(Pose2d target) {
		if (target.getX() != prevTargetPose.getX() || 
			target.getY() != prevTargetPose.getY() || 
			target.getRotation().getDegrees() != prevTargetPose.getRotation().getDegrees()) 
		{ 
			this.driveXPID.reset();
			this.driveYPID.reset();
			this.anglePID.reset();
			this.prevTargetPose = target; 
		}  
		System.out.print("Aligning to pose: ");
        System.out.println(target);

		Pose2d currentPose = this.currentPose();

        double driveXOut = this.driveXPID.calculate(currentPose.getX(), target.getX());
        double driveYOut = this.driveYPID.calculate(currentPose.getY(), target.getY());

        System.out.println("Current Pose: " + this.currentPose());

        System.out.println("Error X: " + driveXPID.getPositionError() + " Error Y: " + driveYPID.getPositionError());

		return new PIDSwerveValues(driveXOut, driveYOut, this.calculatePIDAngleOutput(target.getRotation().getDegrees())); 
	}

	@Override
	public void periodic() {
		// Estimator update
		swerveDrivePoseEstimator.update(navX.getYaw(), getPositions());
		Pose2d odoPose = swerveDrivePoseEstimator.getEstimatedPosition();

		// Vision update
		Optional<EstimatedRobotPose> result = pCameraWrapper.getEstimatedGlobalPose(odoPose);
		if (result.isPresent()) {
			double timestamp = result.get().timestampSeconds;
			Pose2d camPose = result.get().estimatedPose.toPose2d();
			swerveDrivePoseEstimator.addVisionMeasurement(camPose, timestamp);
		}

		// Calculate current speed
		long currentTime = System.currentTimeMillis();
		long deltaTime = currentTime - lastPoseTimestamp;
		currentVelocity = new Translation2d((currentPose().getX() - lastPose.getX()) / (deltaTime / 1000d),
				(currentPose().getY() - lastPose.getY()) / (deltaTime / 1000d));
		lastPose = swerveDrivePoseEstimator.getEstimatedPosition();
		lastPoseTimestamp = System.currentTimeMillis();

		Preferences.getDouble("Swerve DriveX kP", this.XkP);
		Preferences.getDouble("Swerve DriveX kI", this.XkI);
		Preferences.getDouble("Swerve DriveX kD", this.XkD);
		Preferences.getDouble("Swerve DriveY kP", this.YkP);
		Preferences.getDouble("Swerve DriveY kI", this.YkI);
		Preferences.getDouble("Swerve DriveY kD", this.YkD);
		Preferences.getDouble("Swerve Angle kP", this.AkP);
		Preferences.getDouble("Swerve Angle kI", this.AkI);
		Preferences.getDouble("Swerve Angle kD", this.AkD);

		SmartDashboard.putBoolean("Manual Drive Active", manualDrive); 
	}
}

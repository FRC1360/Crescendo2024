// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.units.Power;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStates;
import frc.robot.commands.LEDColorSelect;
import frc.robot.commands.assembly.AssemblySchedulerCommand.ASSEMBLY_LEVEL;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
	private Command m_autonomousCommand;

	private RobotContainer m_robotContainer;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		loggerInit();

		m_robotContainer = new RobotContainer();

		Pathfinding.setPathfinder(new LocalADStar());

		NamedCommands.registerCommand("", m_autonomousCommand);

		m_robotContainer.loadAllAutos();

		m_robotContainer.initalizeAutoChooser();
		SmartDashboard.putString("ALLIANCE",
				DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get().toString() : "NOT AVAIL");
		SmartDashboard.putBoolean("PODIUM_FAR_SCH", false);
		SmartDashboard.putBoolean("PODIUM_LEFT_SCH", false);
		SmartDashboard.putBoolean("PODIUM_RIGHT_SCH", false);
	}

	private void loggerInit() {
		Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

		switch (BuildConstants.DIRTY) {
			case 0:
				Logger.recordMetadata("GitDirty", "All changes committed");
				break;
			case 1:
				Logger.recordMetadata("GitDirty", "Uncomitted changes");
				break;
			default:
				Logger.recordMetadata("GitDirty", "Unknown");
				break;
		}

		if (isReal()) { // real bot
			// Logger.addDataReceiver(new WPILOGWriter());
			Logger.addDataReceiver(new NT4Publisher());
			PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);
			powerDistribution.setSwitchableChannel(true);
		} else if (!Constants.isReplay) { // regular sim
			Logger.addDataReceiver(new NT4Publisher());
		} else { // replay
			setUseTiming(false); // Run as fast as possible
			String logPath = LogFileUtil.findReplayLog();
			Logger.setReplaySource(new WPILOGReader(logPath));
			Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
		}

		AutoLogOutputManager.addPackage("frc.lib");

		Logger.registerURCL(URCL.startExternal());
		Logger.start();
		// DataLogManager.start();
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items
	 * like diagnostics
	 * that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
		// m_robotContainer.loop.poll();
		// System.out.println("Podium scheduled: " + m_robotContainer.LEVEL);
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
		new LEDColorSelect(m_robotContainer.getLedSubsystem(), LEDSubsystem.LEDStates.DISABLED);
	}

	@Override
	public void disabledPeriodic() {
		m_robotContainer.swerveSubsystem.updateAbsAngleSmartDashboard();
		m_robotContainer.swerveSubsystem.resetModuleZeros();
		m_robotContainer.armChassisPivotSubsystem.updateSmartDashboard();
		m_robotContainer.armChassisPivotSubsystem.resetArmTargetAngle();
		m_robotContainer.shintakePivotSubsystem.updateSmartDashboard();
		m_robotContainer.shintakePivotSubsystem.resetSTPTargetAngle();
		// m_robotContainer.shintakePivotSubsystem.resetMotorRotations();
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		// m_robotContainer.swerveSubsystem.setCurrentPose(new Pose2d(12.5, 3.5,
		// Rotation2d.fromDegrees(90.0)));
		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_robotContainer.getHoming()
					.andThen(m_autonomousCommand).schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		m_robotContainer.getHoming().schedule();
		SmartDashboard.putString("ALLIANCE",
				DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get().toString() : "NOT AVAIL");

		// m_robotContainer.shintakePivotSubsystem.setTargetAngle(Constants.HOME_POSITION_STP);
		// m_robotContainer.armChassisPivotSubsystem.setTargetAngle(Constants.HOME_POSITION_ACP);
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		m_robotContainer.pollButtonsForSmartDashboard();

		// m_robotContainer.swerveSubsystem.updateAbsAngleSmartDashboard();
		// m_robotContainer.noteInSource(); // checks if note is in shintake when at
		// source
	}

	private class TestContext {
		public double acpAngle = Constants.HOME_POSITION_ACP;
		public double stpAngle = Constants.HOME_POSITION_STP;
		public double stLeftVel = 0;
		public double stRightVel = 0;
		public double stIntakeVel = 0;
		public double clHeight = 0;
	}

	private TestContext m_test = new TestContext();

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();

		SmartDashboard.putNumber("TEST_ACP_ANGLE", m_test.acpAngle); // Degrees | Min = 0, Max = 80.0
		SmartDashboard.putNumber("TEST_STP_ANGLE", m_test.stpAngle); // Degrees | Min = -180, Max = 360
		SmartDashboard.putNumber("TEST_ST_LEFT_VEL", m_test.stLeftVel); // RPM | limit is 6784
		SmartDashboard.putNumber("TEST_ST_RIGHT_VEL", m_test.stRightVel); // RPM | limit is 6784
		SmartDashboard.putNumber("TEST_INTAKE_VEL", m_test.stIntakeVel); // RPM | limit is 6784
		SmartDashboard.putNumber("TEST_CLIMB_HEIGHT", m_test.clHeight); // Height in Motor rotations | Min and Max
																		// currently unknown
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {

		m_test.acpAngle = SmartDashboard.getNumber("TEST_ACP_ANGLE", m_test.acpAngle);
		m_test.stpAngle = SmartDashboard.getNumber("TEST_STP_ANGLE", m_test.stpAngle);
		m_test.stLeftVel = SmartDashboard.getNumber("TEST_ST_LEFT_VEL", m_test.stLeftVel);
		m_test.stLeftVel = SmartDashboard.getNumber("TEST_ST_RIGHT_VEL", m_test.stRightVel);
		m_test.stIntakeVel = SmartDashboard.getNumber("TEST_INTAKE_VEL", m_test.stIntakeVel);
		m_test.clHeight = SmartDashboard.getNumber("TEST_CLIMB_HEIGHT", m_test.clHeight);

		m_robotContainer.shintakePivotSubsystem.setTargetAngle(m_test.acpAngle);
		m_robotContainer.armChassisPivotSubsystem.setTargetAngle(m_test.stpAngle);
		m_robotContainer.shintakeSubsystem.setVelocity(m_test.stLeftVel, m_test.stRightVel);
		m_robotContainer.shintakeSubsystem.varIntake(m_test.stIntakeVel);
		m_robotContainer.climberSubsystem.setTargetHeight(m_test.clHeight);

		m_robotContainer.shintakePivotSubsystem.periodic();
		m_robotContainer.armChassisPivotSubsystem.periodic();
		m_robotContainer.shintakeSubsystem.periodic();
		m_robotContainer.climberSubsystem.periodic();
	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
	}
}

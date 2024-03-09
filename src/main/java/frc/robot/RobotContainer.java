// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.autos.FetchPath;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ArmChassisPivot.ACPGoToPositionCommand;
import frc.robot.commands.ArmChassisPivot.ACPMoveManual;
import frc.robot.commands.ShintakePivot.STPGoToPositionCommand;
//import frc.robot.commands.ShintakePivot.STPMoveManual;
import frc.robot.commands.assembly.AssemblyAmpPositionCommand;
import frc.robot.commands.assembly.AssemblyHomePositionCommand;
import frc.robot.commands.assembly.AssemblySchedulerCommand;
import frc.robot.commands.assembly.AssemblySourcePositionCommand;
import frc.robot.commands.assembly.AssemblySubwooferPositionCommand;
import frc.robot.commands.assembly.AssemblySchedulerCommand.ASSEMBLY_LEVEL;
import frc.robot.commands.shintake.AmpScoreCommand;
import frc.robot.commands.shintake.IntakeCommand;
import frc.robot.commands.shintake.OutakeCommand;
import frc.robot.commands.shintake.ShootSpeakerCommand;
import frc.robot.commands.shintake.ShootSpeakerFullCommand;
import frc.robot.commands.swerve.LockWheels;
import frc.robot.commands.swerve.RotateForShot;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShintakeSubsystem;
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShintakePivotSubsystem;
import frc.robot.util.StateMachine;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	public ASSEMBLY_LEVEL LEVEL = ASSEMBLY_LEVEL.AMP;
	// The robot's subsystems and commands are defined here...
	public final ShintakePivotSubsystem shintakePivotSubsystem = new ShintakePivotSubsystem();
	public final ArmChassisPivotSubsystem armChassisPivotSubsystem = new ArmChassisPivotSubsystem();

	public final ShintakeSubsystem shintakeSubsystem = new ShintakeSubsystem(); 
	private final LEDSubsystem ledSubsystem = new LEDSubsystem();
	private final StateMachine sm = new StateMachine();
	// Replace with CommandPS4Controller or CommandJoystick if needed

	private final CommandJoystick left_controller = new CommandJoystick(0);
	private final CommandJoystick right_controller = new CommandJoystick(1);
	private final CommandXboxController operator_controller = new CommandXboxController(2);

	public SwerveSubsystem swerveSubsystem;

	public SendableChooser<Command> autoChooser;

	public ArrayList<Command> tempInitAutos;
    public ClimberSubsystem climberSubsystem;

	public InterpolatingDoubleTreeMap shintakePivotDistanceAngleMap; // distance in meters


	// public final EventLoop loop = new EventLoop();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		this.swerveSubsystem = new SwerveSubsystem();

		// m_shintakeSubsystem.setDefaultCommand(m_defaultShintakeCommand);

		// Configure the trigger bindings
		configureBindings();

		this.tempInitAutos = new ArrayList<Command>();
		this.shintakePivotDistanceAngleMap = new InterpolatingDoubleTreeMap(); 

		this.shintakePivotDistanceAngleMap.put(0.5, 50.0);
	}

	public void loadAllAutos() {
		this.tempInitAutos.clear(); // in case if robot is not power cycled, data within class are typically cached
		System.out.println(AutoBuilder.getAllAutoNames());
		for (String pathName : AutoBuilder.getAllAutoNames()) {
			this.tempInitAutos.add(new FetchPath(pathName).getCommand());
		}
	}

	public void initalizeAutoChooser() {
		this.autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData(this.autoChooser);
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
	 * an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link
	 * CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {
		//operator_controller.a().whileTrue(new InstantCommand(()-> armChassisPivotSubsystem.setACPNormalizedVoltage(0.15)));
		//operator_controller.b().whileTrue(new InstantCommand(()-> armChassisPivotSubsystem.setACPNormalizedVoltage(-0.05)));
		// operator_controller.a().onTrue(new ACPGoToPositionCommand(armChassisPivotSubsystem, 60.0));
		// operator_controller.b().onTrue(new ACPGoToPositionCommand(armChassisPivotSubsystem, 30.0));
		// operator_controller.y().onTrue(new ACPGoToPositionCommand(armChassisPivotSubsystem, 50.0));

		//operator_controller.a().whileTrue(new AssemblySubwooferPositionCommand(armChassisPivotSubsystem, shintakePivotSubsystem, ledSubsystem, sm));

		//operator_controller.a().whileTrue(new ACPGoToPositionCommand(armChassisPivotSubsystem, 0.0)); 
		//operator_controller.a().whileTrue(new AssemblyAmpPositionCommand(armChassisPivotSubsystem, shintakePivotSubsystem, ledSubsystem, sm)); 
		//operator_controller.a().whileTrue(new AssemblySourcePositionCommand(armChassisPivotSubsystem, shintakePivotSubsystem, ledSubsystem, sm)); 
		// operator_controller.a().onTrue(new STPGoToPositionCommand(shintakePivotSubsystem, 150.0));
		// operator_controller.b().onTrue(new STPGoToPositionCommand(shintakePivotSubsystem, 60.0));
		operator_controller.y().onTrue(new STPGoToPositionCommand(shintakePivotSubsystem, 90.0, armChassisPivotSubsystem));

		//operator_controller.b().whileTrue(new AssemblyHomePositionCommand(armChassisPivotSubsystem, shintakePivotSubsystem, ledSubsystem, sm)); 
		
		//operator_controller.rightBumper().whileTrue(new IntakeCommand(shintakeSubsystem, ledSubsystem));
		left_controller.button(1).whileTrue(new IntakeCommand(shintakeSubsystem, ledSubsystem)); 

		//operator_controller.x().whileTrue(new OutakeCommand(shintakeSubsystem)); 

		//operator_controller.povUp().whileTrue(new ShootSpeakerCommand(shintakeSubsystem)); 
		//operator_controller.y().whileTrue(new AmpScoreCommand(shintakeSubsystem, armChassisPivotSubsystem, shintakePivotSubsystem, ledSubsystem, sm));
		//operator_controller.povUp().and(() -> this.LEVEL.equals(ASSEMBLY_LEVEL.AMP)).whileTrue(new ShootSpeakerCommand(shintakeSubsystem)); 

		right_controller.button(1)
		.and(() -> this.LEVEL.equals(ASSEMBLY_LEVEL.AMP))
		.whileTrue(new AmpScoreCommand(shintakeSubsystem, ledSubsystem, sm));
		//operator_controller.x().onTrue(new ShootSpeakerFullCommand(shintakeSubsystem, armChassisPivotSubsystem, operator_controller)); 
		right_controller.button(1).and(() -> (this.LEVEL.equals(ASSEMBLY_LEVEL.SUBWOOFER) || this.LEVEL.equals(ASSEMBLY_LEVEL.SUBWOOFER_DEFENDED)))
												.whileTrue(new ShootSpeakerCommand(shintakeSubsystem)); 

		//operator_controller.leftBumper().whileTrue(new InstantCommand(() -> shintakeSubsystem.setVelocity(operator_controller.getLeftTriggerAxis() * 2800, operator_controller.getLeftTriggerAxis() *  2800)));  
		swerveSubsystem.setDefaultCommand(new DefaultDriveCommand(
				swerveSubsystem,
				() -> -modifyAlliance(modifyAxis(left_controller.getY()))
						* Constants.ROBOT_MAX_VELOCITY_METERS_PER_SECOND, // Modify axis also for alliance color
				() -> -modifyAlliance(modifyAxis(left_controller.getX()))
						* Constants.ROBOT_MAX_VELOCITY_METERS_PER_SECOND,
				() -> -modifyAxis(right_controller.getX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
				right_controller));

		left_controller.button(7).onTrue(new
		InstantCommand(swerveSubsystem::zeroGyro));

		// // Left controller Button 1 (trigger) will become shoot (outake)
		// // Right Controller Button 1 (trigger) will be intake
		left_controller.button(2).whileTrue(new InstantCommand(() -> this.LEVEL = ASSEMBLY_LEVEL.SUBWOOFER)
				.andThen(new AssemblySchedulerCommand(() -> this.LEVEL, swerveSubsystem, armChassisPivotSubsystem,
						shintakePivotSubsystem, shintakeSubsystem, ledSubsystem, sm, () -> right_controller.button(3).getAsBoolean(), shintakePivotDistanceAngleMap)));

		left_controller.button(2).whileFalse(new AssemblyHomePositionCommand(armChassisPivotSubsystem, shintakePivotSubsystem, ledSubsystem, sm)); 

		// left_controller.button(3)
		// .whileTrue(new AssemblySchedulerCommand(() -> this.LEVEL, swerveSubsystem, armChassisPivotSubsystem,
		// shintakePivotSubsystem, shintakeSubsystem, ledSubsystem, sm, () -> right_controller.button(3).getAsBoolean())
		// .alongWith(new InstantCommand(() -> System.out.println(this.LEVEL))));

		left_controller.button(4).whileTrue(new InstantCommand(() -> this.LEVEL = ASSEMBLY_LEVEL.AMP)
				.andThen(new AssemblySchedulerCommand(() -> this.LEVEL, swerveSubsystem, armChassisPivotSubsystem,
						shintakePivotSubsystem, shintakeSubsystem, ledSubsystem, sm, () -> right_controller.button(3).getAsBoolean(), shintakePivotDistanceAngleMap)));

		left_controller.button(4).whileFalse(new AssemblyHomePositionCommand(armChassisPivotSubsystem, shintakePivotSubsystem, ledSubsystem, sm)); 

		left_controller.button(5).whileTrue(new InstantCommand(() -> this.LEVEL = ASSEMBLY_LEVEL.SOURCE)
				.andThen(new AssemblySchedulerCommand(() -> this.LEVEL, swerveSubsystem, armChassisPivotSubsystem,
						shintakePivotSubsystem, shintakeSubsystem, ledSubsystem, sm, () -> right_controller.button(3).getAsBoolean(), shintakePivotDistanceAngleMap)));

		left_controller.button(5).whileFalse(new AssemblyHomePositionCommand(armChassisPivotSubsystem, shintakePivotSubsystem, ledSubsystem, sm)); 

		// // left_controller.button(2).whileTrue(new PathfindAuto(swerveSubsystem,
		// // AlignmentConstants.RED_SOURCE).getCommand());

		// // left_controller.button(3).whileTrue(new
		// // PathfindAuto(AlignmentConstants.BLUE_AMP).getCommand());

		// // left_controller.button(4).whileTrue(new PathfindAuto(swerveSubsystem,
		// // AlignmentConstants.BLUE_SPEAKER).getCommand());

		// left_controller.button(7).whileTrue(new LockWheels(swerveSubsystem));
		// right_controller.button(6).whileTrue(new RotateForShot(swerveSubsystem, 
		// 			() -> -modifyAlliance(modifyAxis(left_controller.getY()))
		// 				* Constants.ROBOT_MAX_VELOCITY_METERS_PER_SECOND, // Modify axis also for alliance color
		// 		() -> -modifyAlliance(modifyAxis(left_controller.getX()))
		// 				* Constants.ROBOT_MAX_VELOCITY_METERS_PER_SECOND));

		// right_controller.button(11).onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
		right_controller.button(10).onTrue(new InstantCommand(swerveSubsystem::toggleManualDrive));

		// // Debounce makes for more stability
		// // new BooleanEvent(loop, operator_controller::getYButton).debounce(0.1)
		// // .ifHigh(() -> {this.LEVEL = ASSEMBLY_LEVEL.PODIUM_FAR;
		// // });
		// // new BooleanEvent(loop, operator_controller::getXButton).debounce(0.1)
		// // .ifHigh(() -> {this.LEVEL = ASSEMBLY_LEVEL.PODIUM_LEFT;
		// // });
		// // new BooleanEvent(loop, operator_controller::getBButton).debounce(0.1)
		// // .ifHigh(() -> {this.LEVEL = ASSEMBLY_LEVEL.PODIUM_RIGHT;
		// // });
			

		// // NOTE! The assembly commands will be activated after the driver schedules through assembly scheduler
		// operator_controller.y().onTrue(new InstantCommand(() -> this.LEVEL =
		// ASSEMBLY_LEVEL.PODIUM_FAR));
		// operator_controller.x().onTrue(new InstantCommand(() -> this.LEVEL =
		// ASSEMBLY_LEVEL.PODIUM_LEFT));
		// operator_controller.b().onTrue(new InstantCommand(() -> this.LEVEL =
		// ASSEMBLY_LEVEL.PODIUM_RIGHT));

		// // left_controller.button(7).onTrue(new IntakeCommand(m_shintakeSubsystem));

		// // operator_controller.b().whileTrue(new AssemblySubwooferPositionCommand(armChassisPivotSubsystem, shintakePivotSubsystem, ledSubsystem, sm));
		// // operator_controller.a().whileTrue(new AssemblyAmpPositionCommand(armChassisPivotSubsystem, shintakePivotSubsystem, ledSubsystem, sm));
		// // operator_controller.x().whileTrue(new AssemblySourcePositionCommand(armChassisPivotSubsystem, shintakePivotSubsystem, ledSubsystem, sm));
		// // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

		// operator_controller.leftBumper().whileTrue(new ACPMoveManual(armChassisPivotSubsystem, () -> operator_controller.getRightY())); 
		// operator_controller.rightBumper().whileTrue(new STPMoveManual(shintakePivotSubsystem, () -> operator_controller.getRightY())); 
		// // new Trigger(m_exampleSubsystem::exampleCondition)
		// // .onTrue(new ExampleCommand(m_exampleSubsystem));

		// // Schedule `exampleMethodCommand` when the Xbox controller's B button is
		// // pressed,
		// // cancelling on release.
		// // XboxController.kB.whileTrue(m_exampleSubsystem.exampleMethodCommand());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		//return this.autoChooser.getSelected();
		return new RepeatCommand(new DefaultDriveCommand(swerveSubsystem, () -> -0.25, () -> 0.0, () -> 0.0, right_controller)); 
	}

	private static double deadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0.0;
		}
	}

	private static double modifyAlliance(double value) {
		// Change for based on alliance
		if (DriverStation.getAlliance().isPresent()
				&& DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
			value *= -1;
		}
		return value;
	}

	private static double modifyAxis(double value) {
		// Deadband
		value = deadband(value, 0.05);

		// Square the axis
		value = Math.copySign(value * value, value);

		return value;
	}

	public LEDSubsystem getLedSubsystem() {
		return ledSubsystem;
	}

	public AssemblyHomePositionCommand getHoming() { 
		return new AssemblyHomePositionCommand(armChassisPivotSubsystem, shintakePivotSubsystem, ledSubsystem, sm); 
	}
}
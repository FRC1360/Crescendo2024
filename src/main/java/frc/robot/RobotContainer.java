// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Might need this in the future so don't delete it
//import frc.robot.subsystems.ArmChassisPivotSubsystem;

import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.Shintake.FeedCommand;
import frc.robot.commands.Shintake.FixCommand;
import frc.robot.commands.Shintake.IntakeCommand;
import frc.robot.commands.Shintake.ShootSpeakerCommand;
import frc.robot.subsystems.ShintakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ShintakeSubsystem shintakeSubsystem = new ShintakeSubsystem();
  // public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  //what is this supposed to be :(
  //public final ArmChassisPivotSubsystem ACPSubsystem = new ArmChassisPivotSubsystem(() -> 0.0, () -> false);

  //private final DefaultShintakeCommand m_defaultShintakeCommand = new DefaultShintakeCommand(m_shintakeSubsystem);

  private final IntakeCommand intakeCommand = new IntakeCommand(shintakeSubsystem);
  private final FixCommand fixCommand = new FixCommand(shintakeSubsystem);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick left_controller = new CommandJoystick(0);
  private final CommandJoystick right_controller = new CommandJoystick(1);
  private final CommandXboxController xbox_controller = new CommandXboxController(0);

  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    left_controller.button(1).whileTrue(new IntakeCommand(shintakeSubsystem));
    xbox_controller.a().whileTrue(new ShootSpeakerCommand(shintakeSubsystem));
    xbox_controller.b().whileTrue(new FeedCommand(shintakeSubsystem));
    xbox_controller.x().whileTrue(new IntakeCommand(shintakeSubsystem));
    xbox_controller.y().whileTrue(new FixCommand(shintakeSubsystem));
    left_controller.button(7).onTrue(new IntakeCommand(shintakeSubsystem));

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());



    //swerveSubsystem.setDefaultCommand(new DefaultDriveCommand(
        //swerveSubsystem,
        //() -> -modifyAxis(left_controller.getY()) * Constants.ROBOT_MAX_VELOCITY_METERS_PER_SECOND,
        //() -> -modifyAxis(left_controller.getX()) * Constants.ROBOT_MAX_VELOCITY_METERS_PER_SECOND,
        //() -> modifyAxis(right_controller.getX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        //right_controller));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;//Autos.exampleAuto(m_exampleSubsystem);
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

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}

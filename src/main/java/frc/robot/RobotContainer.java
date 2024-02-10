// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants;
import frc.robot.subsystems.ArmChassisPivotSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ArmChassisPivot.ACPGoToPositionCommand;
import frc.robot.commands.ArmChassisPivot.ACPMoveManual;
import frc.robot.commands.shintake.IntakeCommand;
import frc.robot.subsystems.ShintakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
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
  private final ShintakeSubsystem m_shintakeSubsystem = new ShintakeSubsystem();
  //private final ArmChassisPivotSubsystem m_armChassisPivotSubsystem = new ArmChassisPivotSubsystem(null, null);

  //what is this supposed to be :(
  public final ArmChassisPivotSubsystem ACPSubsystem = new ArmChassisPivotSubsystem(() -> 0.0, () -> false);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick left_controller = new CommandJoystick(0);
  private final CommandJoystick right_controller = new CommandJoystick(1);
  private final CommandXboxController XboxController = new CommandXboxController(2); 
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    ACPSubsystem.setDefaultCommand(new ACPMoveManual(ACPSubsystem, ()-> XboxController.getLeftY()));
    
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

    left_controller.button(7).onTrue(new IntakeCommand(m_shintakeSubsystem));
    XboxController.b().onTrue(new ACPGoToPositionCommand(ACPSubsystem, Constants.ACPConstants.HOME_POSITION_ACP));
    XboxController.a().onTrue(new ACPGoToPositionCommand(ACPSubsystem, Constants.ACPConstants.NOTE_SCORE_AMP_POSITION_ACP));
    XboxController.x().onTrue(new ACPGoToPositionCommand(ACPSubsystem, Constants.ACPConstants.NOTE_SCORE_SPEAKER_POSITION_ACP));
    XboxController.y().onTrue(new ACPGoToPositionCommand(ACPSubsystem, Constants.ACPConstants.SOURCE_POSITION_ACP));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //XboxController.kB.whileTrue(m_exampleSubsystem.exampleMethodCommand());

    swerveSubsystem.setDefaultCommand(new DefaultDriveCommand(
        swerveSubsystem,
        () -> -modifyAxis(left_controller.getY()) * Constants.ROBOT_MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(left_controller.getX()) * Constants.ROBOT_MAX_VELOCITY_METERS_PER_SECOND,
        () -> modifyAxis(right_controller.getX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        right_controller));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return null;
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

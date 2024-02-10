// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.autos.FetchPath;
import frc.robot.autos.PathfindAuto;

import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.assembly.AssemblySchedulerCommand;
import frc.robot.commands.assembly.AssemblySchedulerCommand.ASSEMBLY_LEVEL;
// import frc.robot.commands.shintake.DefaultShintakeCommand;
// import frc.robot.commands.shintake.IntakeCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShintakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  //private final ShintakeSubsystem m_shintakeSubsystem = new ShintakeSubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  //private final DefaultShintakeCommand m_defaultShintakeCommand = new DefaultShintakeCommand(m_shintakeSubsystem);
  
  private final CommandJoystick left_controller = new CommandJoystick(0);
  private final CommandJoystick right_controller = new CommandJoystick(1);
  private final XboxController operator_controller = new XboxController(2); 

  public SwerveSubsystem swerveSubsystem; 
  
  public SendableChooser<Command> autoChooser; 

  public ArrayList<Command> tempInitAutos;
  
  public ASSEMBLY_LEVEL LEVEL = ASSEMBLY_LEVEL.SUBWOOFER; 

  public final EventLoop loop = new EventLoop(); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    this.swerveSubsystem = new SwerveSubsystem(); 

    swerveSubsystem.configureAutoBuilder(); // needs to be called everytime robotInits so alliance is updated

    //m_shintakeSubsystem.setDefaultCommand(m_defaultShintakeCommand);

    // Configure the trigger bindings
    configureBindings();

    this.tempInitAutos = new ArrayList<Command>(); 
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
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //left_controller.button(7).onTrue(new IntakeCommand(m_shintakeSubsystem));

    swerveSubsystem.setDefaultCommand(new DefaultDriveCommand(
        swerveSubsystem,
        () -> -modifyAxis(left_controller.getY()) * Constants.ROBOT_MAX_VELOCITY_METERS_PER_SECOND, // Modify axis also for alliance color
        () -> -modifyAxis(left_controller.getX()) * Constants.ROBOT_MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(right_controller.getX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        right_controller));

    //left_controller.button(1).onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

    // Left controller Button 1 (trigger) will become shoot (outake)
    // Right Controller Button 1 (trigger) will be intake 
    left_controller.button(2).whileTrue(new InstantCommand(() -> this.LEVEL = ASSEMBLY_LEVEL.SUBWOOFER)
                                          .andThen(new AssemblySchedulerCommand(() -> this.LEVEL, swerveSubsystem)));

    left_controller.button(3).whileTrue(new AssemblySchedulerCommand(() -> this.LEVEL, swerveSubsystem).alongWith(new InstantCommand(() -> System.out.println(this.LEVEL)))); 

    left_controller.button(4).whileTrue(new InstantCommand(() -> this.LEVEL = ASSEMBLY_LEVEL.AMP)
                                                  .andThen(new AssemblySchedulerCommand(() -> this.LEVEL, swerveSubsystem)));

    left_controller.button(5).whileTrue(new InstantCommand(() -> this.LEVEL = ASSEMBLY_LEVEL.SOURCE)
                                                  .andThen(new AssemblySchedulerCommand(() -> this.LEVEL, swerveSubsystem)));

    //left_controller.button(2).whileTrue(new PathfindAuto(swerveSubsystem, AlignmentConstants.RED_SOURCE).getCommand()); 

    // left_controller.button(3).whileTrue(new PathfindAuto(AlignmentConstants.BLUE_AMP).getCommand());

    //left_controller.button(4).whileTrue(new PathfindAuto(swerveSubsystem, AlignmentConstants.BLUE_SPEAKER).getCommand()); 

    left_controller.button(7).onTrue(new InstantCommand(swerveSubsystem::brake)); 
    right_controller.button(11).onTrue(new InstantCommand(swerveSubsystem::zeroGyro)); 

    // Debounce makes for more stability
    new BooleanEvent(loop, operator_controller::getYButton).debounce(0.1).ifHigh(() -> this.LEVEL = ASSEMBLY_LEVEL.PODIUM_FAR);
    new BooleanEvent(loop, operator_controller::getXButton).debounce(0.1).ifHigh(() -> this.LEVEL = ASSEMBLY_LEVEL.PODIUM_LEFT);
    new BooleanEvent(loop, operator_controller::getBButton).debounce(0.1).ifHigh(() -> this.LEVEL = ASSEMBLY_LEVEL.PODIUM_RIGHT);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    double x = 15.79;
    double y = 1.90;
    double r = 125.0;
    //Command pathfindAuto = new PathfindAuto(swerveSubsystem, new Pose2d(x, y, Rotation2d.fromDegrees(r))).getCommand();
    // Command pathFollowAuto = new FetchPath("paths/New Path").getCommand();
    return this.autoChooser.getSelected(); 
    
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
    
    // Change for based on alliance
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) { 
      value *= -1; 
    }

    return value;
  }
}

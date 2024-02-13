// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.TimeUnit;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.shintake.FeedCommand;
import frc.robot.commands.shintake.IntakeCommand;
import frc.robot.commands.shintake.ShootSpeakerCommand;
import frc.robot.subsystems.ShintakeSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private ShintakeSubsystem shintakeSubsystem;
  private DigitalInput sensor;
  private Counter counter;
  private double velocity;
  private double count;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    shintakeSubsystem = new ShintakeSubsystem();
    //sensor = new DigitalInput(9);
    //counter = new Counter(sensor);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    velocity = 0.2;
    count = -1;
    shintakeSubsystem.initializeCounter();
    shintakeSubsystem.setVelocity(500, 500);
  }

  

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Intake
      shintakeSubsystem.varIntake(velocity);
      
      //shintakeSubsystem.setVelocity(5500, 5500);
      
    // Pause when optical sensor is tripped
    if (count == shintakeSubsystem.getShintakeCount() && (count != 0)){
      velocity = 0.2;
      shintakeSubsystem.resetShintakeCount();
    }else if (shintakeSubsystem.getShintakeCount() >30000){
      //velocity = 1.0;
      velocity = 0.2; 
    }else if (shintakeSubsystem.isSwitchSet()) {
      velocity = 0.0;
    }

    count = shintakeSubsystem.getShintakeCount();
      
    
    System.out.println("counter:"+shintakeSubsystem.getShintakeCount());
    
      // Wait
      // try {
      //   TimeUnit.SECONDS.sleep(2);
      // } catch (InterruptedException e) {
      //   // TODO Auto-generated catch block
      //   e.printStackTrace();
      // }

      // // Shoot
      // shintakeSubsystem.varIntake(0.0);
      // try {
      //   TimeUnit.SECONDS.sleep(2);
      // } catch (InterruptedException e) {
      //   // TODO Auto-generated catch block
      //   e.printStackTrace();
      // }
      //shintakeSubsystem.setVelocity(0, 30);}
      //shintakeSubsystem.initializeCounter();
}
  



    //IntakeCommand continuousIntake = new IntakeCommand(shintakeSubsystem);
    //ShootSpeakerCommand continuousShoot = new ShootSpeakerCommand(new ShintakeSubsystem());
    //FeedCommand continuousFeed = new FeedCommand(shintakeSubsystem);
    
  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

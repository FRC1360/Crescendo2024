// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.assembly;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ShintakeConstants;
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShintakePivotSubsystem;
import frc.robot.subsystems.ShintakeSubsystem;
import frc.robot.util.StateMachine;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpScoreCommand extends SequentialCommandGroup {

  /** Creates a new AmpScoreCommand. */
  public AmpScoreCommand(ShintakeSubsystem shintakeSubsystem, ArmChassisPivotSubsystem ACPSubsystem,
  ShintakePivotSubsystem STPSubsystem, LEDSubsystem ledSubsystem, StateMachine sm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AssemblyAmpPositionCommand(ACPSubsystem,
    STPSubsystem, ledSubsystem, sm), new InstantCommand(() -> shintakeSubsystem.getShooterReady(shintakeSubsystem.getBackEncoder())), new InstantCommand(() -> shintakeSubsystem.setVelocity(Constants.ShintakeConstants.SHOOT_VELOCITY_FRONT, Constants.ShintakeConstants.SHOOT_VELOCITY_FRONT)));
  }
}

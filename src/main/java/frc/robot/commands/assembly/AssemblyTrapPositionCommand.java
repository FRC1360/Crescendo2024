package frc.robot.commands.assembly;

import java.time.Instant;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmChassisPivot.ACPGoToPositionCommand;
import frc.robot.commands.ShintakePivot.STPGoToPositionCommand;
import frc.robot.commands.shintake.ShootTrapCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShintakePivotSubsystem;
import frc.robot.subsystems.ShintakeSubsystem;
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.util.StateMachine;

public class AssemblyTrapPositionCommand extends SequentialCommandGroup {

    public AssemblyTrapPositionCommand(ArmChassisPivotSubsystem ACPSubsystem,
            ShintakePivotSubsystem STPSubsystem, LEDSubsystem ledSubsystem, StateMachine sm, ShintakeSubsystem shintakeSubsystem) {
        addCommands(
                new InstantCommand(ledSubsystem::setLEDDisable),

                // Command 1
                new ACPGoToPositionCommand(ACPSubsystem, Constants.NOTE_SCORE_TRAP_POSITION_ACP, STPSubsystem)
                        .alongWith(new InstantCommand(() -> SmartDashboard.putString("Trap stage", "STAGE 2")))
                // Command 2
                .alongWith(

                new STPGoToPositionCommand(STPSubsystem, Constants.NOTE_SCORE_TRAP_POSITION_STP, ACPSubsystem)
                        .alongWith(new InstantCommand(() -> SmartDashboard.putString("Trap stage", "STAGE 3")))),
                new ShootTrapCommand(shintakeSubsystem),
                new InstantCommand(ledSubsystem::setLEDEnable),
                new InstantCommand(() -> SmartDashboard.putString("Amp stage", "DONE")),
                new InstantCommand(() -> sm.setAtTrapScore())
                );
    }
}
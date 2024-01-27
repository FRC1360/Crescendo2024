package frc.robot.commands.assembly;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ShintakePivot.STPGoToPositionCommand;
import frc.robot.commands.ArmChassisPivot.ACPGoToPositionCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShintakePivotSubsystem;
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.subsystems.ArmChassisPivotSubsystem.ArmShintakeMessenger;
import frc.robot.util.StateMachine;

public class AssemblyTrapPositionCommand extends SequentialCommandGroup {

    public AssemblyTrapPositionCommand(ArmChassisPivotSubsystem ACPSubsystem,
            ShintakePivotSubsystem STPSubsystem, LEDSubsystem ledSubsystem, StateMachine sm) {
        addCommands(
            new InstantCommand(() -> ACPSubsystem.setInIntakePosition(false)),
            new InstantCommand(ledSubsystem::setLEDDisable),

            // Command 1
            new ACPGoToPositionCommand(ACPSubsystem, Constants.NOTE_SCORE_TRAP_POSITION_ACP)
                .alongWith(new InstantCommand(() -> SmartDashboard.putString("Trap stage", "STAGE 2"))),
            // Command 2

            new STPGoToPositionCommand(STPSubsystem, Constants.NOTE_SCORE_TRAP_POSITION_STP)
                .alongWith(new InstantCommand(() -> SmartDashboard.putString("Trap stage", "STAGE 3"))),
            new InstantCommand(ledSubsystem::setLEDEnable),
            new InstantCommand(() -> SmartDashboard.putString("Amp stage", "DONE")),
            new InstantCommand(() -> sm.setAtClimb())
        );
    }
}

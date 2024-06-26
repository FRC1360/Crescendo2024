package frc.robot.commands.assembly;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmChassisPivot.ACPGoToPositionCommand;
import frc.robot.commands.ShintakePivot.STPGoToPositionCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShintakePivotSubsystem;
import frc.robot.subsystems.ShintakeSubsystem;
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.util.StateMachine;

public class AssemblyAmpPositionCommand extends SequentialCommandGroup {

    public AssemblyAmpPositionCommand(ArmChassisPivotSubsystem ACPSubsystem,
            ShintakePivotSubsystem STPSubsystem, LEDSubsystem ledSubsystem, StateMachine sm) {
        addCommands(
                new InstantCommand(() -> sm.setAtAmpScore()),
                new InstantCommand(ledSubsystem::setLEDDisable),

                // Command 1
                new ACPGoToPositionCommand(ACPSubsystem, Constants.NOTE_SCORE_AMP_POSITION_ACP, STPSubsystem)
                        .alongWith(new InstantCommand(() -> SmartDashboard.putString("Amp stage", "STAGE 2")))//,
                // Command 2

                .alongWith(
                new STPGoToPositionCommand(STPSubsystem, Constants.NOTE_SCORE_AMP_POSITION_STP, ACPSubsystem)
                        .alongWith(new InstantCommand(() -> SmartDashboard.putString("Amp stage", "STAGE 3")))//,
                ),
                new InstantCommand(() -> ledSubsystem.setLEDScoring()),
                new InstantCommand(() -> SmartDashboard.putString("Amp stage", "DONE")),
                new InstantCommand(() -> sm.setAtAmpScore())
                );
    }
}

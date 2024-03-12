package frc.robot.commands.assembly;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmChassisPivot.ACPGoToPositionCommand;
import frc.robot.commands.ShintakePivot.STPGoToPositionCommand;
import frc.robot.commands.shintake.ShootSpeakerFullCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShintakePivotSubsystem;
import frc.robot.subsystems.ShintakeSubsystem;
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.util.StateMachine;

public class AssemblyDefendedArmPositionCommand extends SequentialCommandGroup {

    public AssemblyDefendedArmPositionCommand(ArmChassisPivotSubsystem ACPSubsystem,
            ShintakePivotSubsystem STPSubsystem, LEDSubsystem ledSubsystem, ShintakeSubsystem shintake, StateMachine sm, double targetArmAngle) {
        addCommands(
                new InstantCommand(() -> sm.setAtSpeakerDefendedScore()),
                new InstantCommand(ledSubsystem::setLEDDisable),

                // Command 1
                new ACPGoToPositionCommand(ACPSubsystem, targetArmAngle, STPSubsystem)
                        .alongWith(new InstantCommand(() -> SmartDashboard.putString("Defended stage", "STAGE 2")))
                // Command 2
                .alongWith(
                new STPGoToPositionCommand(STPSubsystem, Constants.NOTE_SCORE_SPEAKER_POSITION_STP_2, ACPSubsystem)
                        .alongWith(new InstantCommand(() -> SmartDashboard.putString("Defended stage", "STAGE 3")))
                ),
                new ShootSpeakerFullCommand(shintake, ACPSubsystem),  
                new InstantCommand(() -> ledSubsystem.setLEDScoring()),
                new InstantCommand(() -> SmartDashboard.putString("Defended stage", "DONE")),
                new InstantCommand(() -> sm.setAtSpeakerDefendedScore()));
    }
}

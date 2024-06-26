package frc.robot.commands.assembly;

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

public class AssemblySubwooferPositionCommand extends SequentialCommandGroup {

        public AssemblySubwooferPositionCommand(ArmChassisPivotSubsystem ACPSubsystem,
                        ShintakePivotSubsystem STPSubsystem, LEDSubsystem ledSubsystem, ShintakeSubsystem shintake,
                        StateMachine sm) {
                if (Constants.SHOOT_ONLY_WITH_ACP) {
                        addCommands(
                                        new InstantCommand(() -> sm.setAtSpeakerSubwooferScore()),
                                        new InstantCommand(ledSubsystem::setLEDDisable),

                                        // Command 1
                                        new ACPGoToPositionCommand(ACPSubsystem,
                                                        Constants.NOTE_SCORE_SPEAKER_POSITION_ACP_2, STPSubsystem)
                                                        .alongWith(new InstantCommand(() -> SmartDashboard
                                                                        .putString("Subwoofer stage", "STAGE 2")))// ,
                                                        // Command 2
                                                        .alongWith(new ShootSpeakerFullCommand(shintake,
                                                                        ACPSubsystem)),
                                        new InstantCommand(ledSubsystem::setLEDScoring),
                                        new InstantCommand(() -> SmartDashboard.putString("Subwoofer stage", "DONE")),
                                        new InstantCommand(() -> sm.setAtSpeakerSubwooferScore()));
                } else {
                        addCommands(
                                        new InstantCommand(() -> sm.setAtSpeakerSubwooferScore()),
                                        new InstantCommand(ledSubsystem::setLEDDisable),

                                        // Command 1
                                        new ACPGoToPositionCommand(ACPSubsystem,
                                                        Constants.NOTE_SCORE_SPEAKER_POSITION_ACP, STPSubsystem)
                                                        .alongWith(new InstantCommand(() -> SmartDashboard
                                                                        .putString("Subwoofer stage", "STAGE 2")))// ,
                                                        // Command 2

                                                        .alongWith(
                                                                        new STPGoToPositionCommand(STPSubsystem,
                                                                                        Constants.NOTE_SCORE_SPEAKER_POSITION_STP,
                                                                                        ACPSubsystem)
                                                                                        .alongWith(new InstantCommand(
                                                                                                        () -> SmartDashboard
                                                                                                                        .putString("Subwoofer stage",
                                                                                                                                        "STAGE 3")))// ,
                                                        ).andThen(
                                                                        new ShootSpeakerFullCommand(shintake,
                                                                                        ACPSubsystem)),
                                        new InstantCommand(() -> ledSubsystem.setLEDScoring()),
                                        new InstantCommand(() -> SmartDashboard.putString("Subwoofer stage", "DONE")),
                                        new InstantCommand(() -> sm.setAtSpeakerSubwooferScore()));
                }
        }
}

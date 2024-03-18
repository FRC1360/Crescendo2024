package frc.robot.commands.assembly;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmChassisPivot.ACPGoToPositionCommand;
import frc.robot.commands.ShintakePivot.STPGoToPositionCommand;
import frc.robot.commands.shintake.ShootSpeakerFullCommand;
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShintakePivotSubsystem;
import frc.robot.subsystems.ShintakeSubsystem;
import frc.robot.util.StateMachine;

public class AssemblyFeedPositionCommand extends SequentialCommandGroup {

    public AssemblyFeedPositionCommand(ArmChassisPivotSubsystem ACPSubsystem,
            ShintakePivotSubsystem STPSubsystem, LEDSubsystem ledSubsystem, ShintakeSubsystem shintake,
            StateMachine sm) {
        addCommands(
                new InstantCommand(() -> sm.setAtSpeakerSubwooferScore()),
                new InstantCommand(ledSubsystem::setLEDDisable),
                new ACPGoToPositionCommand(ACPSubsystem,
                        Constants.NOTE_FEED_POSITION_ACP, STPSubsystem)
                        .alongWith(new InstantCommand(() -> SmartDashboard
                                .putString("Feed stage", "STAGE 2")))
                        .alongWith(new STPGoToPositionCommand(STPSubsystem,
                                Constants.NOTE_FEED_POSITION_STP,
                                ACPSubsystem)
                                .alongWith(new InstantCommand(
                                        () -> SmartDashboard
                                                .putString("Feed stage",
                                                        "STAGE 3"))))
                        .alongWith(
                                new ShootSpeakerFullCommand(shintake,
                                        ACPSubsystem)),
                new InstantCommand(() -> ledSubsystem.setLEDScoring()),
                new InstantCommand(() -> SmartDashboard.putString("Feed stage", "DONE")));
    }

}

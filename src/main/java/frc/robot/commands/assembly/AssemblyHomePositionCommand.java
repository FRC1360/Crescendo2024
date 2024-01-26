package frc.robot.commands.assembly;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.armChassisPivot.ACPGoToPositionCommand;
import frc.robot.commands.shintakePivot.STPGoToPositionCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShintakePivotSubsystem;
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.subsystems.ArmChassisPivotSubsystem.ArmShintakeMessenger;
import frc.robot.util.StateMachine;

public class AssemblyHomePositionCommand extends SequentialCommandGroup {

    public AssemblyHomePositionCommand(ArmChassisPivotSubsystem ACPSubsystem, ArmShintakeMessenger armShintakeMessenger,
            ShintakePivotSubsystem STPSubsystem, LEDSubsystem ledSubsystem, StateMachine sm) {
        addCommands(
            new InstantCommand(() -> ACPSubsystem.setInIntakePosition(false)),
            new InstantCommand(ledSubsystem::setLEDDisable),

            // Command 1
            new STPGoToPositionCommand(STPSubsystem, Constants.HOME_POSITION_STP)
                .alongWith(new InstantCommand(() -> SmartDashboard.putString("Homing stage", "STAGE 2"))),

            // Command 2
            new ACPGoToPositionCommand(ACPSubsystem, Constants.HOME_POSITION_ACP)
                .alongWith(new InstantCommand(() -> SmartDashboard.putString("Homing stage", "STAGE 3"))),

            new InstantCommand(ledSubsystem::setLEDEnable),
            new InstantCommand(() -> SmartDashboard.putString("Homing stage", "DONE")),
            new InstantCommand(() -> sm.setAtHome(true))
        );
    }
}

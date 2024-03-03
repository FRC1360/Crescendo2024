package frc.robot.commands.assembly;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmChassisPivot.ACPGoToPositionCommand;
import frc.robot.commands.ShintakePivot.STPGoToPositionCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShintakePivotSubsystem;
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.util.StateMachine;

public class AssemblyHomePositionCommand extends SequentialCommandGroup {

    public AssemblyHomePositionCommand(ArmChassisPivotSubsystem ACPSubsystem,
            ShintakePivotSubsystem STPSubsystem, LEDSubsystem ledSubsystem, StateMachine sm) {
        addCommands(
                new InstantCommand(() -> sm.setAtHome()),
                new InstantCommand(ledSubsystem::setLEDDisable),

                // Command 1
                new STPGoToPositionCommand(STPSubsystem, Constants.HOME_POSITION_STP, ACPSubsystem)
                        .alongWith(new InstantCommand(() -> SmartDashboard.putString("Homing stage", "STAGE 2"))), 
                // .alongWith(
                // Command 2
                new ACPGoToPositionCommand(ACPSubsystem, Constants.HOME_POSITION_ACP)
                        .alongWith(new InstantCommand(() -> SmartDashboard.putString("Homing stage", "STAGE 3"))), 
                // ), 

                new InstantCommand(ledSubsystem::setLEDEnable),
                new InstantCommand(() -> SmartDashboard.putString("Homing stage", "DONE")),
                new InstantCommand(() -> sm.setAtHome()));
    }
}

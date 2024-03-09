package frc.robot.commands.assembly;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmChassisPivot.ACPGoToPositionCommand;
import frc.robot.commands.ShintakePivot.STPGoToPositionCommand;
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShintakePivotSubsystem;
import frc.robot.util.StateMachine;

public class AssemblyClimbCommand extends SequentialCommandGroup {
    public AssemblyClimbCommand(ArmChassisPivotSubsystem ACPSubsystem,
            ShintakePivotSubsystem STPSubsystem, LEDSubsystem ledSubsystem, StateMachine sm) {
        addCommands(
                new InstantCommand(() -> sm.setAtClimb()),
                new InstantCommand(() -> ledSubsystem.setLEDDisable()),

                // Command 1

                new STPGoToPositionCommand(STPSubsystem, Constants.CLIMB_POSITION_STP, ACPSubsystem)
                        .alongWith(new InstantCommand(() -> SmartDashboard.putString("Climb stage", "STAGE 2")))
                
                .alongWith(
                // Command 2
                new ACPGoToPositionCommand(ACPSubsystem, Constants.CLIMB_POSITION_ACP, STPSubsystem)
                        .alongWith(new InstantCommand(() -> SmartDashboard.putString("Climb stage", "STAGE 3")))
                ), 
                // Command 3 add climber stuff here

                new InstantCommand(() -> ledSubsystem.setLEDEnable()),
                new InstantCommand(() -> SmartDashboard.putString("Climb stage", "DONE")),
                new InstantCommand(() -> sm.setAtAmpScore()));
    }
}

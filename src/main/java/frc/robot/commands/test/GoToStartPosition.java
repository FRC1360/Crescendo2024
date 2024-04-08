package frc.robot.commands.test;

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

public class GoToStartPosition extends SequentialCommandGroup {
    public GoToStartPosition(ArmChassisPivotSubsystem ACPSubsystem,
            ShintakePivotSubsystem STPSubsystem, LEDSubsystem ledSubsystem, StateMachine sm){
                addCommands(new InstantCommand(() -> sm.setAtHome()),
                new InstantCommand(ledSubsystem::setLEDDisable),

                // Command 1
                new STPGoToPositionCommand(STPSubsystem, Constants.START_POSITION_STP, ACPSubsystem)
                        .alongWith(new InstantCommand(() -> SmartDashboard.putString("Start stage", "STAGE 2"))), 
                // .alongWith(
                // Command 2
                new ACPGoToPositionCommand(ACPSubsystem, Constants.START_POSITION_ACP, STPSubsystem)
                        .alongWith(new InstantCommand(() -> SmartDashboard.putString("Start stage", "STAGE 3"))), 
                // ), 

                new InstantCommand(ledSubsystem::setLEDEnable),
                new InstantCommand(() -> SmartDashboard.putString("Start stage", "DONE")),
                new InstantCommand(() -> sm.setAtHome())                  
                );
            }
    
}

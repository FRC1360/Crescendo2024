package frc.robot.commands.assembly;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShintakePivotSubsystem;
import frc.robot.util.StateMachine;

public class AssemblySchedulerCommand extends Command {

    public static enum ASSEMBLY_LEVEL { // the set points that can be set by operator
        SPEAKER,
        AMP
    }

    private Command assemblyCommand;
    private Supplier<ASSEMBLY_LEVEL> level;

    private ArmChassisPivotSubsystem chassisPivot;
    private ShintakePivotSubsystem shintakePivot;
    private LEDSubsystem led;
    private StateMachine sm;
    private int n = 0;

    public AssemblySchedulerCommand(Supplier<ASSEMBLY_LEVEL> level, ArmChassisPivotSubsystem chassisPivot, ShintakePivotSubsystem shintakePivot, LEDSubsystem led, StateMachine sm) {
        this.level = level;
        this.chassisPivot = chassisPivot;
        this.shintakePivot = shintakePivot;
        this.led = led;
        this.sm = sm;
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("SchedCmdN", n++);
        switch(level.get()) {
            case SPEAKER:
                this.assemblyCommand = new AssemblySubwooferPositionCommand(chassisPivot, shintakePivot, led, sm);
                break;
            
            case AMP:
                this.assemblyCommand = new AssemblyAmpPositionCommand(chassisPivot, shintakePivot, led, sm);
                break;

            default:
                break;
        }

        this.assemblyCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return this.assemblyCommand.isFinished();
    }
    
    
}

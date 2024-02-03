package frc.robot.commands.assembly;

import java.sql.Driver;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignmentConstants;
import frc.robot.Constants.Swerve;
import frc.robot.autos.PathfindAuto;
import frc.robot.subsystems.SwerveSubsystem;

public class AssemblySchedulerCommand extends Command {

    public static enum ASSEMBLY_LEVEL { // the set points that can be set by operator
        PODIUM_LEFT,
        PODIUM_FAR, 
        PODIUM_RIGHT, 
        SUBWOOFER,
        AMP,
        SOURCE
    }

    private Command assemblyCommand;
    private SwerveSubsystem swerveSubsystem; 
    private Supplier<ASSEMBLY_LEVEL> level;

    public AssemblySchedulerCommand(Supplier<ASSEMBLY_LEVEL> level, SwerveSubsystem swerveSubsystem) {
        this.level = level;
        this.swerveSubsystem = swerveSubsystem; 
    }

    @Override
    public void initialize() {
        //SmartDashboard.putNumber("SchedCmdN", n++);
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) { 
            switch(level.get()) {
                case PODIUM_LEFT:
                    //this.assemblyCommand = new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot, led, sm);
                    this.assemblyCommand = new PathfindAuto(swerveSubsystem, AlignmentConstants.BLUE_STAGE_LEFT).getCommand(); 
                    break;

                case PODIUM_RIGHT: 
                    this.assemblyCommand = new PathfindAuto(this.swerveSubsystem, AlignmentConstants.BLUE_STAGE_RIGHT).getCommand(); 

                case PODIUM_FAR: 
                    this.assemblyCommand = new PathfindAuto(this.swerveSubsystem, AlignmentConstants.BLUE_STAGE_FAR).getCommand();

                case SUBWOOFER:
                    //this.assemblyCommand = new AssemblySubwooferPositionCommand(chassisPivot, shintakePivot, led, sm);
                    this.assemblyCommand = new PathfindAuto(swerveSubsystem, AlignmentConstants.BLUE_SPEAKER).getCommand(); 
                    break;
                
                case AMP:
                    //this.assemblyCommand = new AssemblyAmpPositionCommand(chassisPivot, shintakePivot, led, sm);
                    this.assemblyCommand = new PathfindAuto(swerveSubsystem, AlignmentConstants.BLUE_AMP).getCommand(); 
                    break;

                case SOURCE:
                    //this.assemblyCommand = new AssemblySourcePositionCommand(chassisPivot, shintakePivot, led, sm);
                    this.assemblyCommand = new PathfindAuto(swerveSubsystem, AlignmentConstants.BLUE_SOURCE).getCommand(); 
                    break;
                default:
                    break;
            }
        }
        else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) { 
             switch(level.get()) {
                case PODIUM_LEFT:
                    //this.assemblyCommand = new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot, led, sm);
                    this.assemblyCommand = new PathfindAuto(swerveSubsystem, AlignmentConstants.RED_STAGE_LEFT).getCommand(); 
                    break;

                case PODIUM_RIGHT: 
                    this.assemblyCommand = new PathfindAuto(this.swerveSubsystem, AlignmentConstants.RED_STAGE_RIGHT).getCommand(); 

                case PODIUM_FAR: 
                    this.assemblyCommand = new PathfindAuto(this.swerveSubsystem, AlignmentConstants.RED_STAGE_FAR).getCommand();

                case SUBWOOFER:
                    //this.assemblyCommand = new AssemblySubwooferPositionCommand(chassisPivot, shintakePivot, led, sm);
                    this.assemblyCommand = new PathfindAuto(swerveSubsystem, AlignmentConstants.RED_SPEAKER).getCommand(); 
                    break;
                
                case AMP:
                    //this.assemblyCommand = new AssemblyAmpPositionCommand(chassisPivot, shintakePivot, led, sm);
                    this.assemblyCommand = new PathfindAuto(swerveSubsystem, AlignmentConstants.RED_AMP).getCommand(); 
                    break;

                case SOURCE:
                    //this.assemblyCommand = new AssemblySourcePositionCommand(chassisPivot, shintakePivot, led, sm);
                    this.assemblyCommand = new PathfindAuto(swerveSubsystem, AlignmentConstants.RED_SOURCE).getCommand(); 
                    break;
                default:
                    break;
            }
        }

        this.assemblyCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return this.assemblyCommand.isFinished();
    }
    
    
}

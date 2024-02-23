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
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShintakePivotSubsystem;
import frc.robot.util.StateMachine;

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
    private ArmChassisPivotSubsystem chassisPivot;
    private ShintakePivotSubsystem shintakePivot;
    private LEDSubsystem led;
    private StateMachine sm;
    private int n = 0;

    public AssemblySchedulerCommand(Supplier<ASSEMBLY_LEVEL> level, SwerveSubsystem swerveSubsystem,
            ArmChassisPivotSubsystem chassisPivot, ShintakePivotSubsystem shintakePivot, LEDSubsystem led,
            StateMachine sm) {
        this.level = level;
        this.swerveSubsystem = swerveSubsystem;
        this.chassisPivot = chassisPivot;
        this.shintakePivot = shintakePivot;
        this.led = led;
        this.sm = sm;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("SCHEDULER GOING TO", level.get().name());
        // SmartDashboard.putNumber("SchedCmdN", n++);
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            switch (level.get()) {
                case PODIUM_LEFT:
                    // this.assemblyCommand = new AssemblyPodiumPositionCommand(chassisPivot,
                    // shintakePivot, led, sm);
                    this.assemblyCommand = new PathfindAuto(swerveSubsystem, AlignmentConstants.BLUE_STAGE_LEFT)
                            .getCommand()
                            .andThen(new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;

                case PODIUM_RIGHT:
                    this.assemblyCommand = new PathfindAuto(this.swerveSubsystem, AlignmentConstants.BLUE_STAGE_RIGHT)
                            .getCommand()
                            .andThen(new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;

                case PODIUM_FAR:
                    this.assemblyCommand = new PathfindAuto(this.swerveSubsystem, AlignmentConstants.BLUE_STAGE_FAR)
                            .getCommand()
                            .andThen(new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;

                case SUBWOOFER:
                    // this.assemblyCommand = new AssemblySubwooferPositionCommand(chassisPivot,
                    // shintakePivot, led, sm);
                    this.assemblyCommand = new PathfindAuto(swerveSubsystem, AlignmentConstants.BLUE_SPEAKER)
                            .getCommand()
                            .andThen(new AssemblySubwooferPositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;

                case AMP:
                    // this.assemblyCommand = new AssemblyAmpPositionCommand(chassisPivot,
                    // shintakePivot, led, sm);
                    this.assemblyCommand = new PathfindAuto(swerveSubsystem, AlignmentConstants.BLUE_AMP).getCommand()
                            .andThen(new AssemblyAmpPositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;

                case SOURCE:
                    // this.assemblyCommand = new AssemblySourcePositionCommand(chassisPivot,
                    // shintakePivot, led, sm);
                    this.assemblyCommand = new PathfindAuto(swerveSubsystem, AlignmentConstants.BLUE_SOURCE)
                            .getCommand()
                            .andThen(new AssemblySourcePositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;
                default:
                    break;
            }
        } else if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            switch (level.get()) {
                case PODIUM_LEFT:
                    // this.assemblyCommand = new AssemblyPodiumPositionCommand(chassisPivot,
                    // shintakePivot, led, sm);
                    this.assemblyCommand = new PathfindAuto(swerveSubsystem, AlignmentConstants.RED_STAGE_LEFT)
                            .getCommand()
                            .andThen(new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;

                case PODIUM_RIGHT:
                    this.assemblyCommand = new PathfindAuto(this.swerveSubsystem, AlignmentConstants.RED_STAGE_RIGHT)
                            .getCommand()
                            .andThen(new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;

                case PODIUM_FAR:
                    this.assemblyCommand = new PathfindAuto(this.swerveSubsystem, AlignmentConstants.RED_STAGE_FAR)
                            .getCommand()
                            .andThen(new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;

                case SUBWOOFER:
                    // this.assemblyCommand = new AssemblySubwooferPositionCommand(chassisPivot,
                    // shintakePivot, led, sm);
                    this.assemblyCommand = new PathfindAuto(swerveSubsystem, AlignmentConstants.RED_SPEAKER)
                            .getCommand()
                            .andThen(new AssemblySubwooferPositionCommand(chassisPivot, shintakePivot, led, sm));
                    ;
                    break;

                case AMP:
                    // this.assemblyCommand = new AssemblyAmpPositionCommand(chassisPivot,
                    // shintakePivot, led, sm);
                    this.assemblyCommand = new PathfindAuto(swerveSubsystem, AlignmentConstants.RED_AMP).getCommand()
                            .andThen(new AssemblyAmpPositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;

                case SOURCE:
                    // this.assemblyCommand = new AssemblySourcePositionCommand(chassisPivot,
                    // shintakePivot, led, sm);
                    this.assemblyCommand = new PathfindAuto(swerveSubsystem, AlignmentConstants.RED_SOURCE).getCommand()
                            .andThen(new AssemblySourcePositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;
                default:
                    break;
            }
        }

        this.assemblyCommand.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        this.assemblyCommand.cancel();
    }

    @Override
    public boolean isFinished() {
        return this.assemblyCommand.isFinished();
    }

}

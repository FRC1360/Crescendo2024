package frc.robot.commands.assembly;

import java.sql.Driver;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.AlignmentConstants;
import frc.robot.Constants.Swerve;
import frc.robot.autos.PathfindAuto;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShintakePivotSubsystem;
import frc.robot.subsystems.ShintakeSubsystem;
import frc.robot.util.StateMachine;

public class AssemblySchedulerCommand extends Command {

    public static enum ASSEMBLY_LEVEL { // the set points that can be set by operator
        PODIUM_LEFT,
        PODIUM_FAR,
        PODIUM_RIGHT,
        SUBWOOFER,
        SUBWOOFER_DEFENDED, 
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
    private ShintakeSubsystem shintake; 
    private BooleanSupplier noPathFind; 
    private InterpolatingDoubleTreeMap shintakePivotDistanceMap; 
    private int n = 0;

    public AssemblySchedulerCommand(Supplier<ASSEMBLY_LEVEL> level, SwerveSubsystem swerveSubsystem,
            ArmChassisPivotSubsystem chassisPivot, ShintakePivotSubsystem shintakePivot, ShintakeSubsystem shintake, LEDSubsystem led,
            StateMachine sm, BooleanSupplier noPathfind, InterpolatingDoubleTreeMap shintakePivotDistanceMap) {
        this.level = level;
        this.swerveSubsystem = swerveSubsystem;
        this.chassisPivot = chassisPivot;
        this.shintakePivot = shintakePivot;
        this.led = led;
        this.shintake = shintake; 
        this.noPathFind = noPathfind; 
        this.sm = sm;
        this.shintakePivotDistanceMap = shintakePivotDistanceMap; 
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("SCHEDULER GOING TO", level.get().name());
        SmartDashboard.putBoolean("With pathfinding", !noPathFind.getAsBoolean());
        // SmartDashboard.putNumber("SchedCmdN", n++);
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            switch (level.get()) {
                case PODIUM_LEFT:
                    // this.assemblyCommand = new AssemblyPodiumPositionCommand(chassisPivot,
                    // shintakePivot, led, sm);
                    this.assemblyCommand = new ConditionalCommand(new InstantCommand(), new PathfindAuto(swerveSubsystem, AlignmentConstants.BLUE_STAGE_LEFT).getCommand(), noPathFind)
                            .andThen(new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;

                case PODIUM_RIGHT:
                    this.assemblyCommand = new ConditionalCommand(new InstantCommand(), new PathfindAuto(this.swerveSubsystem, AlignmentConstants.BLUE_STAGE_RIGHT).getCommand(), noPathFind)
                            .andThen(new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;

                case PODIUM_FAR:
                    this.assemblyCommand = new ConditionalCommand(new InstantCommand(), new PathfindAuto(this.swerveSubsystem, AlignmentConstants.BLUE_STAGE_FAR)
                            .getCommand(), noPathFind)
                            .andThen(new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;

                case SUBWOOFER:
                    // this.assemblyCommand = new AssemblySubwooferPositionCommand(chassisPivot,
                    // shintakePivot, led, sm);
                    this.assemblyCommand = 
                    // new ConditionalCommand(
                    //     new InstantCommand(), new PathfindAuto(swerveSubsystem, AlignmentConstants.BLUE_SPEAKER).getCommand(),
                    //     noPathFind)
                            //.andThen(
                                new AssemblySubwooferPositionCommand(chassisPivot, shintakePivot, led, shintake, sm); 
                                //);
                    break;

                case AMP:
                    // this.assemblyCommand = new AssemblyAmpPositionCommand(chassisPivot,
                    // shintakePivot, led, sm);
                    this.assemblyCommand = 
                    // new ConditionalCommand(new InstantCommand(), new PathfindAuto(swerveSubsystem, AlignmentConstants.BLUE_AMP).getCommand(), noPathFind)
                    //         .andThen(
                                new AssemblyAmpPositionCommand(chassisPivot, shintakePivot, led, sm); 
                                // );
                    break;

                case SOURCE:
                    // this.assemblyCommand = new AssemblySourcePositionCommand(chassisPivot,
                    // shintakePivot, led, sm);
                    this.assemblyCommand = 
                    // new ConditionalCommand(new InstantCommand(), new PathfindAuto(swerveSubsystem, AlignmentConstants.BLUE_SOURCE)
                    //          .getCommand(), noPathFind)
                           // .andThen(
                                new AssemblySourcePositionCommand(chassisPivot, shintakePivot, led, sm);
                                //);
                    break;
                case SUBWOOFER_DEFENDED: 
                    this.assemblyCommand = new AssemblyDefendedPositionCommand(chassisPivot, shintakePivot, led, shintake, sm, 
                                                shintakePivotDistanceMap.get(swerveSubsystem.calculateDistanceToTarget(AlignmentConstants.BLUE_SPEAKER))); 
                    break; 
                default:
                    break;
            }
        } else if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            switch (level.get()) {
                case PODIUM_LEFT:
                    this.assemblyCommand = new ConditionalCommand(new InstantCommand(), new PathfindAuto(this.swerveSubsystem, AlignmentConstants.RED_STAGE_LEFT)
                            .getCommand(), noPathFind)
                            .andThen(new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;

                case PODIUM_RIGHT:
                    this.assemblyCommand = new ConditionalCommand(new InstantCommand(), new PathfindAuto(this.swerveSubsystem, AlignmentConstants.RED_STAGE_RIGHT)
                            .getCommand(), noPathFind)
                            .andThen(new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;

                case PODIUM_FAR:
                    this.assemblyCommand = new ConditionalCommand(new InstantCommand(), new PathfindAuto(this.swerveSubsystem, AlignmentConstants.RED_STAGE_FAR)
                            .getCommand(), noPathFind)
                            .andThen(new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;

                case SUBWOOFER:
                    // this.assemblyCommand = new AssemblySubwooferPositionCommand(chassisPivot,
                    // shintakePivot, led, sm);
                    this.assemblyCommand = new ConditionalCommand(new InstantCommand(), new PathfindAuto(swerveSubsystem, AlignmentConstants.RED_SPEAKER).getCommand(), noPathFind)
                            .andThen(new AssemblySubwooferPositionCommand(chassisPivot, shintakePivot, led, shintake, sm));
                    break;

                case AMP:
                    // this.assemblyCommand = new AssemblyAmpPositionCommand(chassisPivot,
                    // shintakePivot, led, sm);
                    this.assemblyCommand = new ConditionalCommand(new InstantCommand(), new PathfindAuto(swerveSubsystem, AlignmentConstants.RED_AMP).getCommand(), noPathFind)
                            .andThen(new AssemblyAmpPositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;

                case SOURCE:
                    // this.assemblyCommand = new AssemblySourcePositionCommand(chassisPivot,
                    // shintakePivot, led, sm);
                    this.assemblyCommand = new ConditionalCommand(new InstantCommand(), new PathfindAuto(swerveSubsystem, AlignmentConstants.RED_SOURCE).getCommand(), noPathFind)
                            .andThen(new AssemblySourcePositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;

                case SUBWOOFER_DEFENDED: 
                    this.assemblyCommand = new AssemblyDefendedPositionCommand(chassisPivot, shintakePivot, led, shintake, sm, 
                                                this.shintakePivotDistanceMap.get(this.swerveSubsystem.calculateDistanceToTarget(AlignmentConstants.RED_SPEAKER))); 
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

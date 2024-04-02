package frc.robot.commands.assembly;

import java.sql.Driver;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import javax.swing.GroupLayout.Alignment;

import com.ctre.phoenix.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.AlignmentConstants;
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
        SUBWOOFER, // fixed shot with shintake pivot
        SUBWOOFER_DEFENDED, // the Kevin Durant shot from the back
        SUBWOOFER_ARM, // subwoofer shot using arm
        AMP,
        SOURCE,
        TRAP
    }

    public static enum SOURCE_SIDE {
        LEFT,
        CENTER,
        RIGHT
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
    private int n = 0;

    public AssemblySchedulerCommand(Supplier<ASSEMBLY_LEVEL> level, SwerveSubsystem swerveSubsystem,
            ArmChassisPivotSubsystem chassisPivot, ShintakePivotSubsystem shintakePivot, ShintakeSubsystem shintake,
            LEDSubsystem led,
            StateMachine sm, BooleanSupplier noPathfind) {
        this.level = level;
        this.swerveSubsystem = swerveSubsystem;
        this.chassisPivot = chassisPivot;
        this.shintakePivot = shintakePivot;
        this.led = led;
        this.shintake = shintake;
        this.noPathFind = noPathfind;
        this.sm = sm;
    }

    private Command conditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        // Implements witb no add requirements for command that isn't running
        return condition.getAsBoolean() ? onTrue : onFalse;
    }

    // private double calculateArmAngle(double distanceToCenter) {
    // double f = 5.5 * (2.54 / 100);
    // double y = 2.10 - (9 * (2.54 / 100));
    // double x = distanceToCenter + (9.75 * (2.54 / 100));
    // double d = Math.hypot(x, y);

    // double theta = Math.toDegrees(Math.atan(y / x)) + Math.toDegrees(Math.acos(f
    // / d)) - 90;
    // theta = theta + (43 - theta) * 0.5;
    // return theta;
    // }

    private double getXVel() {
        Pose2d velocity = swerveSubsystem.currentVelocity;
        double theta = swerveSubsystem.currentPose().getRotation().getRadians();
        return Math.sin(theta) * velocity.getY() + Math.cos(theta) * velocity.getX();
    }

    private double getYVel() {
        Pose2d velocity = swerveSubsystem.currentVelocity;
        double theta = swerveSubsystem.currentPose().getRotation().getRadians();
        return Math.sin(theta) * velocity.getX() + Math.cos(theta) * velocity.getY();
    }

    public double calculateArmAngle(double distanceToCenter) {
        double NOTE_VELOCITY = 10;
        double TARGET_HEIGHT = Units.inchesToMeters(86);
        double SHOOTER_HEIGHT = Units.inchesToMeters(8.75);
        double dx = distanceToCenter + (9.75 * (2.54 / 100));
        double dy = 2.10 - (9 * (2.54 / 100));
        double distance = Math.hypot(dx, dy);

        double y = TARGET_HEIGHT - SHOOTER_HEIGHT;
        double flight_time = // distance / NOTE_VELOCITY; // USE THIS IF NO SHOOT ON MOVE
                distance / (NOTE_VELOCITY + (Math.sqrt(getXVel() * getXVel() + getYVel() * getYVel())));
        // * MathUtil.clamp(shintake.getVelocityLeft() / shintake.leftVelocity, 0.25,
        // 1);
        y += 9.8 / 2 * flight_time * flight_time;
        SmartDashboard.putNumber("Angle", Units.radiansToDegrees(Math.atan(y / distance)));
        SmartDashboard.putNumber("Distance", distance);
        return (Units.radiansToDegrees(Math.atan(y / distance)));
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
                    this.assemblyCommand = conditionalCommand(new InstantCommand(),
                            new PathfindAuto(swerveSubsystem, AlignmentConstants.BLUE_STAGE_LEFT).getCommand(),
                            noPathFind);
                    // .andThen(new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot, led,
                    // sm));
                    break;

                case PODIUM_RIGHT:
                    this.assemblyCommand = conditionalCommand(new InstantCommand(),
                            new PathfindAuto(this.swerveSubsystem, AlignmentConstants.BLUE_STAGE_RIGHT).getCommand(),
                            noPathFind);
                    // .andThen(new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot, led,
                    // sm));
                    break;

                case PODIUM_FAR:
                    this.assemblyCommand = conditionalCommand(new InstantCommand(),
                            new PathfindAuto(this.swerveSubsystem, AlignmentConstants.BLUE_STAGE_FAR)
                                    .getCommand(),
                            noPathFind);
                    // .andThen(new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot, led,
                    // sm));
                    break;

                case SUBWOOFER:
                    this.assemblyCommand = conditionalCommand(
                            new InstantCommand(),
                            new PathfindAuto(swerveSubsystem, AlignmentConstants.BLUE_SPEAKER, true).getCommand(),
                            noPathFind)
                            .alongWith(
                                    new AssemblySubwooferPositionCommand(chassisPivot, shintakePivot, led, shintake,
                                            sm));
                    break;

                case AMP:
                    this.assemblyCommand = conditionalCommand(new InstantCommand(),
                            new PathfindAuto(swerveSubsystem, AlignmentConstants.BLUE_AMP, true).getCommand(),
                            noPathFind)
                            .alongWith(
                                    new AssemblyAmpPositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;

                case SOURCE:
                    this.assemblyCommand =
                            // conditionalCommand(new InstantCommand(), new PathfindAuto(swerveSubsystem,
                            // AlignmentConstants.BLUE_SOURCE_CENTER, true)
                            // .getCommand(), noPathFind)
                            // .alongWith(
                            new AssemblySourcePositionCommand(chassisPivot, shintakePivot, led, sm);
                    // );
                    break;
                case SUBWOOFER_DEFENDED:
                    // based on lookup table to calculate shintake angle
                    this.assemblyCommand = new AssemblyDefendedPositionCommand(chassisPivot, shintakePivot, led,
                            shintake,
                            sm,
                            shintakePivot.shintakePivotDistanceAngleMap.get(
                                    swerveSubsystem.calculateDistanceToTarget(AlignmentConstants.INTO_BLUE_SPEAKER)));
                    SmartDashboard.putNumber("Target angle",
                            shintakePivot.shintakePivotDistanceAngleMap.get(
                                    swerveSubsystem.calculateDistanceToTarget(AlignmentConstants.INTO_BLUE_SPEAKER)));

                    break;
                case SUBWOOFER_ARM:
                    // based on trig to calculate arm angle
                    this.assemblyCommand = new AssemblyDefendedArmPositionCommand(chassisPivot,
                            shintakePivot, led, shintake, sm,
                            () -> calculateArmAngle(swerveSubsystem
                                    .calculateDistanceToTarget(AlignmentConstants.INTO_BLUE_SPEAKER)));

                    break;
                // case TRAP:
                // new AssemblyAmpPositionCommand(chassisPivot, shintakePivot, led, sm));
                // break;

                case TRAP:
                    this.assemblyCommand = new AssemblyTrapPositionCommand(chassisPivot, shintakePivot, led, sm,
                            shintake);
                    break;

                default:
                    break;
            }
        } else if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            switch (level.get()) {
                case PODIUM_LEFT:
                    this.assemblyCommand = conditionalCommand(new InstantCommand(),
                            new PathfindAuto(this.swerveSubsystem, AlignmentConstants.RED_STAGE_LEFT)
                                    .getCommand(),
                            noPathFind);
                    // .alongWith(new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot,
                    // led, sm));
                    break;

                case PODIUM_RIGHT:
                    this.assemblyCommand = conditionalCommand(new InstantCommand(),
                            new PathfindAuto(this.swerveSubsystem, AlignmentConstants.RED_STAGE_RIGHT)
                                    .getCommand(),
                            noPathFind);
                    // .alongWith(new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot,
                    // led, sm));
                    break;

                case PODIUM_FAR:
                    this.assemblyCommand = conditionalCommand(new InstantCommand(),
                            new PathfindAuto(this.swerveSubsystem, AlignmentConstants.RED_STAGE_FAR)
                                    .getCommand(),
                            noPathFind);
                    // .alongWith(new AssemblyPodiumPositionCommand(chassisPivot, shintakePivot,
                    // led, sm));
                    break;

                case SUBWOOFER:
                    this.assemblyCommand = conditionalCommand(new InstantCommand(),
                            new PathfindAuto(swerveSubsystem, AlignmentConstants.RED_SPEAKER).getCommand(), noPathFind)
                            .alongWith(new AssemblySubwooferPositionCommand(chassisPivot, shintakePivot, led, shintake,
                                    sm));
                    break;

                case AMP:
                    this.assemblyCommand = conditionalCommand(new InstantCommand(),
                            new PathfindAuto(swerveSubsystem, AlignmentConstants.RED_AMP).getCommand(), noPathFind)
                            .alongWith(new AssemblyAmpPositionCommand(chassisPivot, shintakePivot, led, sm));
                    break;

                case SOURCE:
                    this.assemblyCommand = // new ConditionalCommand(new InstantCommand(), new
                                           // PathfindAuto(swerveSubsystem, AlignmentConstants.RED_SOURCE_CENTER,
                                           // true).getCommand(), noPathFind);
                            // .alongWith(
                            new AssemblySourcePositionCommand(chassisPivot, shintakePivot, led, sm);
                    // );
                    break;
                case SUBWOOFER_DEFENDED:
                    // this.assemblyCommand = new RepeatCommand(new
                    // AssemblyDefendedPositionCommand(chassisPivot, shintakePivot, led, shintake,
                    // sm,
                    // //
                    // shintakePivot.shintakePivotDistanceAngleMap.get(this.swerveSubsystem.calculateDistanceToTarget(AlignmentConstants.RED_SPEAKER))));

                    this.assemblyCommand = new RepeatCommand(
                            new AssemblyDefendedArmPositionCommand(chassisPivot, shintakePivot, led,
                                    shintake, sm,
                                    () -> calculateArmAngle(swerveSubsystem
                                            .calculateDistanceToTarget(AlignmentConstants.INTO_RED_SPEAKER))));
                    break;

                case SUBWOOFER_ARM:
                    // based on trig to calculate arm angle
                    this.assemblyCommand = new AssemblyDefendedArmPositionCommand(chassisPivot,
                            shintakePivot, led, shintake, sm,
                            () -> calculateArmAngle(swerveSubsystem
                                    .calculateDistanceToTarget(AlignmentConstants.INTO_BLUE_SPEAKER)));
                    break;

                case TRAP:
                    this.assemblyCommand = new AssemblyTrapPositionCommand(chassisPivot, shintakePivot, led, sm,
                            shintake);
                    break;

                default:
                    break;
            }
        }

        this.assemblyCommand.schedule();
    }

    @Override
    public void execute() {
        // if (level.get() == ASSEMBLY_LEVEL.SUBWOOFER_DEFENDED) {
        // chassisPivot.setTargetAngle(calculateArmAngle(this.swerveSubsystem.calculateDistanceToTarget(AlignmentConstants.INTO_BLUE_SPEAKER)));
        // }
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

package frc.robot.commands.ArmChassisPivot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.subsystems.ShintakePivotSubsystem;

public class ACPGoToPositionCommand extends Command {

    ArmChassisPivotSubsystem ACP;
    ShintakePivotSubsystem STP;
    double angle;
    boolean initalized = false;

    BooleanSupplier needsDelay;

    public ACPGoToPositionCommand(ArmChassisPivotSubsystem ACP, double angle, ShintakePivotSubsystem STPSubsystem,
            BooleanSupplier needsDelay) {
        this.ACP = ACP;
        this.STP = STPSubsystem;
        this.angle = angle;
        this.needsDelay = needsDelay;
        addRequirements(ACP);
    }

    public ACPGoToPositionCommand(ArmChassisPivotSubsystem ACP, double angle, ShintakePivotSubsystem STPSubsystem) {
        this(ACP, angle, STPSubsystem, () -> false);
    }

    @Override
    public void initialize() {
        initalized = false;
        // this.ACP.setTargetAngle(angle);

        // System.out.println("Shoulder angle set to: " + this.ACP.getTargetAngle());
    }

    @Override
    public void execute() {
        // System.out.println("Delay: " + needsDelay.getAsBoolean());
        if (!initalized && this.STP.getSTPAngle() < 80.0) {
            // if (needsDelay.getAsBoolean() && this.STP.getSTPAngle() < 80.0) {
            this.ACP.setTargetAngle(angle);
            initalized = true;
            // } else if (!needsDelay.getAsBoolean()) {
            // this.ACP.setTargetAngle(angle);
            // initalized = true;
            // }
        }
    }

    @Override
    public boolean isFinished() {
        return this.ACP.atTarget() && initalized;
    }

    @Override
    public void end(boolean interrupted) {
    }
}

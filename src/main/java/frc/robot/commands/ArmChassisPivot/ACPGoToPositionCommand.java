package frc.robot.commands.ArmChassisPivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.subsystems.ShintakePivotSubsystem;

public class ACPGoToPositionCommand extends Command {

    ArmChassisPivotSubsystem ACP;
    ShintakePivotSubsystem STP;
    double angle;
    boolean initalized = false;

    public ACPGoToPositionCommand(ArmChassisPivotSubsystem ACP, double angle, ShintakePivotSubsystem STPSubsystem) {
        this.ACP = ACP;
        this.STP = STPSubsystem;
        this.angle = angle;
        addRequirements(ACP);
    }

    @Override
    public void initialize() {
        initalized = false;
        // this.ACP.setTargetAngle(angle);

        // System.out.println("Shoulder angle set to: " + this.ACP.getTargetAngle());
    }

    @Override
    public void execute() {
        if (this.STP.getSTPAngle() < 120.0 && !initalized) {
            this.ACP.setTargetAngle(angle);
            initalized = true;
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

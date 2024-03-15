package frc.robot.commands.ShintakePivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.subsystems.ShintakePivotSubsystem;

public class STPGoToPositionCommand extends Command {

    private ShintakePivotSubsystem ShintakePivot;
    private double angle;
    private ArmChassisPivotSubsystem arm;
    private boolean initalized = false;

    public STPGoToPositionCommand(ShintakePivotSubsystem ShintakePivot, double angle, ArmChassisPivotSubsystem arm) {
        this.ShintakePivot = ShintakePivot;
        this.angle = angle;
        this.arm = arm;
        addRequirements(ShintakePivot);
    }

    @Override
    public void initialize() {
        initalized = false;
        // System.out.println("STP angle set to: " +
        // this.ShintakePivot.getTargetAngle());
        // this.ShintakePivot.setTargetAngle(angle);
    }

    @Override
    public void execute() {
        if (/* angle >= 80.0 && this.arm.getACPAngle() > 20.0 && */ !initalized) {
            this.ShintakePivot.setTargetAngle(angle);
            initalized = true;
        }
    }

    @Override
    public boolean isFinished() {
        return this.ShintakePivot.atTarget() && initalized;
    }
}
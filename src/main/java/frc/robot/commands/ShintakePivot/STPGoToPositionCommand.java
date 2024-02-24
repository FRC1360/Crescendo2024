package frc.robot.commands.ShintakePivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShintakePivotSubsystem;

public class STPGoToPositionCommand extends Command {

    private ShintakePivotSubsystem ShintakePivot;
    private double angle; 

    public STPGoToPositionCommand(ShintakePivotSubsystem ShintakePivot, double angle) {
        this.ShintakePivot = ShintakePivot;
        this.angle = angle;
        addRequirements(ShintakePivot);
    }

    @Override
    public void initialize() {
        this.ShintakePivot.setTargetAngle(angle); 

        System.out.println("STP angle set to: " + this.ShintakePivot.getTargetAngle());
    }

    @Override
    public boolean isFinished() {
        return this.ShintakePivot.atTarget(); 
    }
}
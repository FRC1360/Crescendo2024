package frc.robot.commands.ArmChassisPivot;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmChassisPivotSubsystem; 

public class ACPGoToPositionCommand extends Command {

    ArmChassisPivotSubsystem ACP; 
    double angle; 
    
    public ACPGoToPositionCommand(ArmChassisPivotSubsystem ACP, double angle) {
        this.ACP = ACP;
        this.angle = angle; 
        addRequirements(ACP);
    }

    @Override
    public void initialize() {
        this.ACP.setTargetAngle(angle);

        System.out.println("Shoulder angle set to: " + this.ACP.getTargetAngle());
    }

    @Override
    public boolean isFinished() { 
        return this.ACP.atTarget(); 
    }

    @Override
    public void end(boolean interrupted) {
    }
}

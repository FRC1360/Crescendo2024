package frc.robot.commands.shintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShintakeSubsystem;

public class MoveBackNoteCommand extends Command {
    
    ShintakeSubsystem shintakeSubsystem; 
    double prev = -1; 
    boolean finished = false; 
    public MoveBackNoteCommand(ShintakeSubsystem shintake) { 
        this.shintakeSubsystem = shintake; 
        addRequirements(shintake);
    }

    @Override
    public void initialize() { 
        prev = this.shintakeSubsystem.getBackEncoder(); 
    }

    @Override
    public void execute() {
        finished = this.shintakeSubsystem.getShooterReady(prev); 
    }

    @Override
    public boolean isFinished() { 
        return finished; 
    }


}

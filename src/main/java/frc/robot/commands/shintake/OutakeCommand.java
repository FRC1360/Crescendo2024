package frc.robot.commands.shintake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShintakeSubsystem;

public class OutakeCommand extends Command {

    private ShintakeSubsystem intake;
    // private int m_count;

    public OutakeCommand(ShintakeSubsystem intake) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // m_count = 0;
        intake.stopShooter();
        intake.stopIntake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // intake.varShoot(-Constants.ShintakeConstants.UNFEED_SPEED_BACK);
        intake.varFix(Constants.ShintakeConstants.OUTAKE_SPEED_BACK);
        intake.varShoot(Constants.ShintakeConstants.OUTAKE_SPEED_FRONT);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
        intake.stopShooter();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

package frc.robot.commands.driverAssist;
import frc.robot.autos.PathfindAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class LineupSource extends Command {
    public LineupSource() {}

  // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void initialize() {
        double x = 1.24;  // TODO TUNE THESE VALUES
        double y = 1.70;  // TODO TUNE THESE VALUES
        double r = 120.0; // TODO TUNE THESE VALUES

        Command pathfindAuto = new PathfindAuto(new Pose2d(x, y, Rotation2d.fromDegrees(r))).getCommand();
        pathfindAuto.schedule();
    } 

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}

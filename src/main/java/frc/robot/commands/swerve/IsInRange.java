package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

public class IsInRange extends Command {
    private Pose2d target;
    private Supplier<Pose2d> position;
    private double positionTolerance, angleTolerance;

    public IsInRange(Supplier<Pose2d> position, Pose2d target, double positionTolerance, double angleTolerance) {
        this.position = position;
        this.target = target;
        this.positionTolerance = positionTolerance;
        this.angleTolerance = angleTolerance;
    }

    @Override
    public boolean isFinished() {
        Transform2d error = target.minus(position.get());
        return error.getTranslation().getNorm() < positionTolerance
                && error.getRotation().getRadians() < angleTolerance;
    }
}

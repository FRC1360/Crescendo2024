package frc.robot.commands.ShintakePivot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShintakePivotSubsystem;

public class STPMoveManual extends Command {

    private final DoubleSupplier joystick;
    private final ShintakePivotSubsystem shintakePivot;

    public STPMoveManual(ShintakePivotSubsystem shintakePivot, DoubleSupplier joystick) {
        this.shintakePivot = shintakePivot;
        this.joystick = joystick;

        addRequirements(shintakePivot);
    }

    @Override
    public void execute() {
        double joystickValue = joystick.getAsDouble();
        double adjustedAngle = shintakePivot.getTargetAngle() + joystickValue * 0.5; // Adjust as needed
        shintakePivot.setTargetAngle(adjustedAngle);
        SmartDashboard.putNumber("shintakePivot_Raw_Output", joystickValue * 0.5);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

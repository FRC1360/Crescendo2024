package frc.robot.commands.armChassisPivot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmChassisPivotSubsystem;

public class ACPMoveManual extends Command {

    private final DoubleSupplier joystick;
    private final ArmChassisPivotSubsystem armChassisPivot;

    public ACPMoveManual(ArmChassisPivotSubsystem armChassisPivot, DoubleSupplier joystick) {
        this.armChassisPivot = armChassisPivot;
        this.joystick = joystick;

        addRequirements(armChassisPivot);
    }

    @Override
    public void execute() {
        double joystickValue = joystick.getAsDouble();
        double adjustedAngle = armChassisPivot.getTargetAngle() + joystickValue * 0.5; // Adjust as needed
        armChassisPivot.setTargetAngle(adjustedAngle);
        SmartDashboard.putNumber("ArmChassisPivot_Raw_Output", joystickValue * 0.5);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

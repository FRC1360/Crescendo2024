package frc.robot.commands.ShintakePivot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShintakePivotSubsystem;

public class STPMoveManual extends Command {

    private final DoubleSupplier joystick;
    private final ShintakePivotSubsystem shintakePivot;
    private CommandXboxController xboxController; 

    private double curTarget; 

    public STPMoveManual(ShintakePivotSubsystem shintakePivot, DoubleSupplier joystick, CommandXboxController xboxController) {
        this.shintakePivot = shintakePivot;
        this.joystick = joystick;
        this.xboxController = xboxController; 

        addRequirements(shintakePivot);
    }

    @Override
    public void initialize() { 
        curTarget = shintakePivot.getTargetAngle();
    }

    @Override
    public void execute() {
        double joystickValue = joystick.getAsDouble();
        double offset = joystickValue * 30;
        double adjustedAngle = curTarget + offset; // Adjust as needed
        shintakePivot.setTargetAngle(adjustedAngle);
        SmartDashboard.putNumber("shintakePivot_Raw_Output", offset);

        if (xboxController.a().getAsBoolean()) {
            System.out.println("Setting relative STP offset to: " + offset); 
            this.shintakePivot.setCacheOffset(offset);
        }

        shintakePivot.setTargetAngle(curTarget);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

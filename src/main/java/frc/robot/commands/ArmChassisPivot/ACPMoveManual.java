package frc.robot.commands.ArmChassisPivot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmChassisPivotSubsystem;

public class ACPMoveManual extends Command {

    private final DoubleSupplier joystick;
    private final ArmChassisPivotSubsystem armChassisPivot;
    private CommandXboxController xboxController; 

    private double curTarget; 

    public ACPMoveManual(ArmChassisPivotSubsystem armChassisPivot, DoubleSupplier joystick, CommandXboxController xboxController) {
        this.armChassisPivot = armChassisPivot;
        this.joystick = joystick;
        this.xboxController = xboxController; 

        addRequirements(armChassisPivot);
    }

    @Override
    public void initialize() { 
        curTarget = armChassisPivot.getACPAngle(); 
    }

    @Override
    public void execute() {
        double joystickValue = joystick.getAsDouble();
        double offset = joystickValue * 10; 
        //double adjustedAngle = curTarget + offset; //* 0.5; // Adjust as needed
        //armChassisPivot.setTargetAngle(adjustedAngle);
        SmartDashboard.putNumber("ArmChassisPivot_Raw_Output", offset);

        if (xboxController.a().getAsBoolean()) {
            System.out.println("Setting ACP offset to: " + offset); 
            this.armChassisPivot.setCacheOffset(offset);
        }

        armChassisPivot.setTargetAngle(curTarget);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
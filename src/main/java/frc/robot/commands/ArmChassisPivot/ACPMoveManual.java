package frc.robot.commands.ArmChassisPivot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmChassisPivotSubsystem;

public class ACPMoveManual extends Command {

    DoubleSupplier joystick;
    ArmChassisPivotSubsystem armchassispivot;

    public ACPMoveManual(ArmChassisPivotSubsystem armchassispivot, DoubleSupplier joystick) {
        this.armchassispivot = armchassispivot;
        this.joystick = joystick;

        addRequirements(armchassispivot);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        this.armchassispivot.setACPNormalizedVoltage(joystick.getAsDouble());
        SmartDashboard.putNumber("ArmChassisPivot_Raw_Output", joystick.getAsDouble() * 0.5);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDStates;

public class LEDColorSelect extends Command {

    private final LEDSubsystem ledSubsystem;
    private final LEDStates ledState;

    public LEDColorSelect(LEDSubsystem ledSubsystem, LEDStates ledState) {
        this.ledSubsystem = ledSubsystem;
        this.ledState = ledState;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        switch (ledState) {
            case ENABLED:
                ledSubsystem.setLEDEnable();
                break;
            case DISABLED:
                ledSubsystem.setLEDDisable();
                break;
            case NOTE:
                ledSubsystem.setLEDNote();
                break;
            case SOURCE:
                ledSubsystem.setLEDSource();
                break;
            case SCORING:
                ledSubsystem.setLEDScoring();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }
}

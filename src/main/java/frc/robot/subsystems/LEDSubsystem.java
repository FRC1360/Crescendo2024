package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase{

    
    private Spark LEDController;
    private double LEDColour;
    public LEDStates LEDstate;

    public enum LEDStates {
        DISABLED, ENABLED, SCORING, SOURCE, NOTE
    }
    
    
    public LEDSubsystem() {
        this.LEDController = new Spark(Constants.LED.LEDPort); 
        this.LEDColour = 0.0;
        this.LEDstate = LEDStates.ENABLED;
    }

    public void setLEDEnable() {
        this.LEDstate = LEDStates.ENABLED;
    }

    public void setLEDDisable() {
        this.LEDstate = LEDStates.DISABLED;
    }

    public void setLEDSource() {
        this.LEDstate = LEDStates.SOURCE;
    }

    public void setLEDScoring() {
        this.LEDstate = LEDStates.SCORING;
    }

    public void setLEDNote() {
        this.LEDstate = LEDStates.NOTE;
    }

    @Override
    public void periodic() {
        switch(this.LEDstate) {
            case ENABLED:
                this.LEDColour = Constants.LED.ENABLED_COLOR;
                break;

            case DISABLED: 
                this.LEDColour = Constants.LED.DISABLED_COLOR;
                break;

            case NOTE:
                this.LEDColour = Constants.LED.NOTE_COLOR;
                break;

            case SOURCE:
                this.LEDColour = Constants.LED.SOURCE_COLOR;
                break;

            case SCORING:
                this.LEDColour = Constants.LED.SCORING_COLOR;
                break;
                
        }

        this.LEDController.set(this.LEDColour);
    }

}
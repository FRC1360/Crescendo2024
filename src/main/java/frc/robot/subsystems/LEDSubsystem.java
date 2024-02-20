package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase{

    
    private Spark LEDController;
    private double LEDColour;
    private LEDStates LEDstate;

    public enum LEDStates {
        DISABLED, ENABLED, SCORING, SOURCE, NOTE
    }
    
    
    public LEDSubsystem() {
        this.LEDController = new Spark(Constants.LEDPort); 
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
                this.LEDColour = 0.87; // BLUE
                break;

            case DISABLED: 
                this.LEDColour = 0.77; // GREEN
                break;

            case NOTE:
                this.LEDColour = 0.6; // ORANGE
                break;

            case SOURCE:
                this.LEDColour = 0.93; // WHITE 
                break;

            case SCORING:
                this.LEDColour = 0.61; // RED
                break;
                
        }

        this.LEDController.set(this.LEDColour);
    }

}
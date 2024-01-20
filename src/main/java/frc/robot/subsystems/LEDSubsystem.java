package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{

    private Spark LEDController;
    private double LEDColour;
    private LEDStates LEDstate;

    public enum LEDStates {
        ENABLED, DISABLED, FOLDED, SOURCE, AMP, SUBWOOFER_SPEAKER, PODIUM_SPEAKER, CLIMBING
    }
    
    
    public LEDSubsystem() {
        this.LEDController = new Spark(0);  // TODO - Change PWM port
        this.LEDColour = 0.0;
        this.LEDstate = LEDStates.ENABLED;
    }

    public void setLEDEnable() {
        this.LEDstate = LEDStates.ENABLED;
    }

    public void setLEDDisable() {
        this.LEDstate = LEDStates.DISABLED;
    }

    public void setLEDFOLDED() {
        this.LEDstate = LEDStates.FOLDED;
    }

    public void setLEDSOURCE() {
        this.LEDstate = LEDStates.SOURCE;
    }

    public void setLEDAMP() {
        this.LEDstate = LEDStates.AMP;
    }
    public void setLEDSUBWOOFER_SPEAKER(){
        this.LEDstate = LEDStates.SUBWOOFER_SPEAKER;
    }
    public void setLEDPODIUM_SPEAKER(){
        this.LEDstate = LEDStates.PODIUM_SPEAKER;
    }
    public void setLEDCLIMBING(){
        this.LEDstate = LEDStates.CLIMBING;
    }

    @Override
    public void periodic() {
        switch(this.LEDstate) {
            case ENABLED:
                this.LEDColour = 0; //tbd
                break;
            case DISABLED:  
                this.LEDColour = 0; //tbd
                break;
            case FOLDED:
                this.LEDColour = 0.6; // ORANGE(If in possesion of note)
                break;
            case SOURCE:
                this.LEDColour = 0.65; //ORANGE
                break;
            case AMP:
                this.LEDColour = 0.61; // RED
                break;
            case SUBWOOFER_SPEAKER:
                this.LEDColour = 0.69; // Yellow
                break;
            case CLIMBING:
                this.LEDColour = 0.91; // VIOLET
                break;
            case PODIUM_SPEAKER:
                this.LEDColour = 0.77; // GREEN
                break;
                
        }

        this.LEDController.set(this.LEDColour);
    }

}
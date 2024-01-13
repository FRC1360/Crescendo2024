package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{

    private Spark LEDController;
    private double LEDColour;
    private LEDStates LEDstate;

    public enum LEDStates {
        ENABLED, DISABLED, CONE, CUBE, SPECIAL
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

    public void setLEDCone() {
        this.LEDstate = LEDStates.CONE;
    }

    public void setLEDCube() {
        this.LEDstate = LEDStates.CUBE;
    }

    public void setLEDSpecial() {
        this.LEDstate = LEDStates.SPECIAL;
    }

    @Override
    public void periodic() {
        switch(this.LEDstate) {
            case ENABLED:
                this.LEDColour = 0.95; // GRAY
                break;
            case DISABLED:
                this.LEDColour = 0.61; // RED
                break;
            case CONE:
                this.LEDColour = 0.69; // YELLOW
                break;
            case CUBE:
                this.LEDColour = 0.91; // VIOLET
                break;
            case SPECIAL:
                this.LEDColour = 0.73; // LIME
        }

        this.LEDController.set(this.LEDColour);
    }

}
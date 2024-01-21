package frc.robot.util;

public class StateMachine {

    boolean atHome;
    boolean atSpeakerScore;
    boolean atAmpScore;
    boolean atIntake;
    boolean atClimb;

    public StateMachine() {
        atHome = false;
    }

    public void setAtHome(boolean atHome) {
        this.atHome = atHome;
    }

    public boolean getAtHome() {
        return this.atHome;
    }
    
}
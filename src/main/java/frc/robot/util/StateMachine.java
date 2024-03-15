package frc.robot.util;

public class StateMachine {

    public enum ArmPositionStates {
        atHome, atSpeakerSubwooferScore, atSpeakerPodiumScore, atSpeakerDefendedScore, atAmpScore, atSource, atClimb
    }

    public ArmPositionStates armPosState;

    public StateMachine() {
        this.armPosState = ArmPositionStates.atHome;
    }

    public void setAtHome() {
        this.armPosState = ArmPositionStates.atHome;
    }

    public void setAtSpeakerSubwooferScore() {
        this.armPosState = ArmPositionStates.atSpeakerSubwooferScore;
    }

    public void setAtSpeakerPodiumScore() {
        this.armPosState = ArmPositionStates.atSpeakerPodiumScore;
    }

    public void setAtSpeakerDefendedScore() {
        this.armPosState = ArmPositionStates.atSpeakerDefendedScore;
    }

    public void setAtAmpScore() {
        this.armPosState = ArmPositionStates.atAmpScore;
    }

    public void setAtClimb() {
        this.armPosState = ArmPositionStates.atClimb;
    }

    public void setAtSource() {
        this.armPosState = ArmPositionStates.atSource;
    }

}
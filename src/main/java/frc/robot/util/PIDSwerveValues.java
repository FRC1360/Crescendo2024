package frc.robot.util;

public class PIDSwerveValues {
    // Used to transport calculated PID values to commands

    public double xOut; public double yOut; public double rotationOut; 
    
    public PIDSwerveValues(double xOut, double yOut, double rotationOut) { 
        this.xOut = xOut; 
        this.yOut = yOut; 
        this.rotationOut = rotationOut; 
    }
}

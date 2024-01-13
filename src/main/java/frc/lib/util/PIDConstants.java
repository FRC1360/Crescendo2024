package frc.lib.util;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class for managing the PID controllers and be able to make changes that
 * affect all loops in the robot such as live edit
 */
public class PIDConstants {
    public double p, i, d;
    public double tolerance = 0;
    private String subscript;

    public PIDConstants(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public PIDConstants(double p, double i, double d, double tolerance) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.tolerance = tolerance;
    }

    public PIDController getController() {
        PIDController controller = new PIDController(this.p, this.i, this.d);
        if (tolerance != 0) {
            controller.setTolerance(tolerance);
        }
        return controller;
    }

    public PIDController getControllerP() {
        PIDController controller = new PIDController(p, 0, d);
        if (tolerance != 0) {
            controller.setTolerance(tolerance);
        }
        return controller;
    }

    /**
     * Applies the pid values for the given controller
     * 
     * @param controller Motor controller, ex. SparkMaxPIDController
     */
    public void applyPID(SparkMaxPIDController controller) {
        controller.setP(this.p);
        controller.setI(this.i);
        controller.setD(this.d);
    }

    /**
     * Sending the values to the dashboard with the given subscript
     * 
     * @param subscript The subscript to put behind the values to be able to
     *                  differentiate
     */
    public void sendDashboard(String subscript) {
        this.subscript = subscript;
        SmartDashboard.putNumber(subscript + " P", this.p);
        SmartDashboard.putNumber(subscript + " I", this.i);
        SmartDashboard.putNumber(subscript + " D", this.d);
    }

    public void retrieveDashboard() {
        this.p = SmartDashboard.getNumber(this.subscript + " P", 0.0);
        this.i = SmartDashboard.getNumber(this.subscript + " I", 0.0);
        this.d = SmartDashboard.getNumber(this.subscript + " D", 0.0);
    }

    /**
     * Get the values from the dashboard and update the controller pid values in
     * case they have been changed by the user in the dashboard
     * 
     * @param controller Motor controller, ex. SparkMaxPIDController
     */
    public void retrieveDashboard(SparkMaxPIDController controller) {
        double p = SmartDashboard.getNumber(this.subscript + " P", 0.0);
        double i = SmartDashboard.getNumber(this.subscript + " I", 0.0);
        double d = SmartDashboard.getNumber(this.subscript + " D", 0.0);

        if (this.p != p) {
            controller.setP(p);
            this.p = p;
        }
        if (this.i != i) {
            controller.setI(i);
            this.i = i;
        }
        if (this.d != d) {
            controller.setD(d);
            this.d = d;
        }
    }

    /**
     * Get the values from the dashboard and update the controller pid values in
     * case they have been changed by the user in the dashboard
     * 
     * @param controller
     * @return The controller with the updated values
     */
    public PIDController retrieveDashboard(PIDController controller) {
        double p = SmartDashboard.getNumber(this.subscript + " P", 0.0);
        double i = SmartDashboard.getNumber(this.subscript + " I", 0.0);
        double d = SmartDashboard.getNumber(this.subscript + " D", 0.0);

        if (this.p != p) {
            controller.setP(p);
            this.p = p;
        }
        if (this.i != i) {
            controller.setI(i);
            this.i = i;
        }
        if (this.d != d) {
            controller.setD(d);
            this.d = d;
        }

        return controller;
    }
}
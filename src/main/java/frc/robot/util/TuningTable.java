// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class TuningTable {
    private String key;
    private double defaultValue;
    private double lastValue = defaultValue;

    /** Constructor */
    public TuningTable(String key) {
        this.key = key;
    }

    /** set default value of the number */
    public void setDefault(double defaultValue) {
        this.defaultValue = defaultValue;
        SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
    }

    /** get the current value */
    public double get() {
        return SmartDashboard.getNumber(key, defaultValue);
    }

    /** check whether the value has been changed since last check */
    public boolean hasChanged() {
        double currentValue = get();
        if (currentValue != lastValue) {
            lastValue = currentValue;
            return true;
        } else {
            return false;
        }
    }
}
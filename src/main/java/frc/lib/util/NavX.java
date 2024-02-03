package frc.lib.util;

import org.littletonrobotics.junction.AutoLogOutput;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NavX {
    // gyro values
    private AHRS gyro;
    private float yawOffset, pitchOffset, rollOffset;
    private boolean inverted = false;

    public NavX() {
        // Gyro setup
        try {
            gyro = new AHRS(SPI.Port.kMXP);
            yawOffset = pitchOffset = rollOffset = 0;
        } catch (RuntimeException exception) {
            DriverStation.reportError("Error initializing NavX: " + exception.getMessage(), true);
        }
    }

    public void setInverted(boolean isInverted) {
        inverted = isInverted;
    }

    public boolean isInverted() {
        return inverted;
    }

    @AutoLogOutput(key = "Swerve/NavX/Yaw")
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(
                inverted
                        ? 360 - (gyro.getFusedHeading() - yawOffset)
                        : gyro.getFusedHeading() - yawOffset);
    }

    @AutoLogOutput(key = "Swerve/NavX/Pitch")
    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch() - pitchOffset);
    }

    @AutoLogOutput(key = "Swerve/NavX/Roll")
    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll() - rollOffset);
    }

    public void resetGyro() {
        yawOffset = gyro.getFusedHeading();
        pitchOffset = gyro.getPitch();
        rollOffset = gyro.getRoll();
        System.out.println("NavX Reset"); 
    }

    public void updateSD() {
        SmartDashboard.putNumber("NavX raw Yaw", gyro.getFusedHeading());
        SmartDashboard.putNumber("Yaw offset", yawOffset);
    }

}

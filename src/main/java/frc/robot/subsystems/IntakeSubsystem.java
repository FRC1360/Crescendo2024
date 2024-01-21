package frc.robot.subsystems; 

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    
    private CANSparkMax leadMotor; // the motor to open and close the claw
    private CANSparkMax followingMotor; // the motor that follows the lead motor
    // public DigitalInput intakeSensor; 
    // public DigitalInput in2; 
    // public DigitalInput in3; 
    // public DigitalInput in4; 
    private boolean atSubstation; 

    public IntakeSubsystem() { 
        this.leadMotor = new CANSparkMax(Constants.LEAD_INTAKE_MOTOR_ID, MotorType.kBrushless);
        this.followingMotor = new CANSparkMax(Constants.FOLLOW_INTAKE_MOTOR_ID, MotorType.kBrushless); 
        this.atSubstation = false; 

        followingMotor.follow(leadMotor);  
    }

    public void stop(){
        leadMotor.stopMotor();
        SmartDashboard.putNumber("Intake motor speed", 0.0); 
    }

    public void intake(double speed){
        leadMotor.set(speed);
        //SmartDashboard.putNumber("Intake motor speed", speed); 
        //SmartDashboard.putBoolean("Intaked object", this.intakeSensor.get()); 
    }

    public void setAtSubstationState(boolean state) { 
        this.atSubstation = state; 
    }

    public boolean getAtSubstationState() { 
        return this.atSubstation; 
    }

}
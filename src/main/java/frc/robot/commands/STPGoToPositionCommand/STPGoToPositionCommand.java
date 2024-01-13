package frc.robot.commands.STPGoToPositionCommand;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.OrbitTimer;

public class STPGoToPositionCommand extends CommandBase {
    
    private WristSubsystem wrist;
    private double angle;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State startState;
    private TrapezoidProfile.State endState;

    private OrbitTimer timer;

    public STPGoToPositionCommand(WristSubsystem wrist, double angle) {
        this.wrist = wrist;
        this.angle = angle;
        this.timer = new OrbitTimer();
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        this.wrist.movePIDController.reset();
        this.wrist.setWristOffset(angle);

        double startVelocity = 0.0;

        if (!this.wrist.getAngularVelocity().isNaN()) { 
            startVelocity = this.wrist.getAngularVelocity().doubleValue(); 
        } 

        this.startState = new TrapezoidProfile.State(this.wrist.getWristAngle(), startVelocity);
        //this.endState = new TrapezoidProfile.State(this.wrist.getTargetAngle(), 0.0);
        this.endState = new TrapezoidProfile.State(this.wrist.getWristOffset(), 0.0);

        this.motionProfile = new TrapezoidProfile(this.wrist.wristMotionProfileConstraints, 
            this.endState,
            this.startState);
        
        this.timer.start();

        System.out.println("Set Wrist Offset = " + this.angle);
    }

    @Override
    public void execute() {
        TrapezoidProfile.State profileTarget = this.motionProfile.calculate(this.timer.getTimeDeltaSec());

        double target = profileTarget.position;
        double input = this.wrist.getWristAngle();

        SmartDashboard.putNumber("Wrist_Move_Profile_Position", profileTarget.position);
        SmartDashboard.putNumber("Wrist_Move_Profile_Velocity", profileTarget.velocity);

        double pidOutput = this.wrist.movePIDController.calculate(target, input);
        //SmartDashboard.putNumber("Wrist_Motion_Profile_Ouput", pidOutput);

        //if (Math.abs(speed) > 0.50) speed =  Math.copySign(0.5, speed); 
        double feedforwardOutput = 0.0;
        
        if (!this.wrist.getAngularVelocity().isNaN()) { 
            feedforwardOutput = this.wrist.wristFeedForward.calculate(Math.toRadians(profileTarget.position), 
                                                                        Math.toRadians(this.wrist.getAngularVelocity())); 
        } 

        double speed = pidOutput + feedforwardOutput; 

        if(Math.abs(speed) > 0.4) {
            speed = Math.copySign(0.4, speed);  // Clamping speed to prevent motor stall
        }

        // SmartDashboard.putNumber("Wrist_Move_PID_Output", pidOutput);
        // SmartDashboard.putNumber("Wrist_FF_Output", feedforwardOutput);
        SmartDashboard.putNumber("Wrist_Move_PID_And_FF_Output", speed); 

        this.wrist.setWristNormalizedVoltage(speed);
    }

    @Override
    public boolean isFinished() {
        return this.motionProfile.isFinished(this.timer.getTimeDeltaSec())  
            /*&& Math.abs(this.wrist.getWristAngle() - this.wrist.getTargetAngle()) < 3.0*/;
    }
}
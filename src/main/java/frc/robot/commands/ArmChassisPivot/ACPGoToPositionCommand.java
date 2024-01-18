package frc.robot.commands.ArmChassisPivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.util.OrbitTimer;

public class ACPGoToPositionCommand extends Command {
    
    private ArmChassisPivotSubsystem shoulder;
    private double angle;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State startState;
    private TrapezoidProfile.State endState;

    private OrbitTimer timer;

    public ACPGoToPositionCommand(ArmChassisPivotSubsystem shoulder, double angle) {
        this.shoulder = shoulder;
        this.angle = angle;
        this.timer = new OrbitTimer();
        addRequirements(shoulder);
    }

    @Override
    public void initialize() {
        this.shoulder.movePIDController.reset();
        this.shoulder.setTargetAngle(angle);

        System.out.println("Shoulder angle set to: " + this.shoulder.getTargetAngle()); 
        
        this.startState = new TrapezoidProfile.State(this.shoulder.getACPAngle(), 0.0);
        this.endState = new TrapezoidProfile.State(this.shoulder.getTargetAngle(), 0.0);

        //what do i do with ACPUpMotionProfileConstraints & ACPDownMotionProfileConstraints

        this.timer.start();

    }

    @Override
    public void execute() {
        
        TrapezoidProfile.State profileTarget;

        if (this.shoulder.getACPAngle() - this.shoulder.getTargetAngle() < 0) { 
            // Going up
            profileTarget = motionProfile.calculate(this.timer.getTimeDeltaSec(),
                this.endState,
                this.startState);
        }
        else { 
            // Going down or stationary
            profileTarget = motionProfile.calculate(this.timer.getTimeDeltaSec(),
                this.endState,
                this.startState);
        }

        //TrapezoidProfile.State profileTarget = this.motionProfile.calculate(this.timer.getTimeDeltaSec());

        double target = profileTarget.position;

        SmartDashboard.putNumber("Shoulder_Move_Profile_Position", profileTarget.position);
        SmartDashboard.putNumber("Shoulder_Move_Profile_Velocity", profileTarget.velocity);

        double input = this.shoulder.getACPAngle();

        double pidOutput = this.shoulder.movePIDController.calculate(target, input);

        double feedforwardOutput = this.shoulder.ACPFeedForward
                                    .calculate(target, this.shoulder.getAngluarVelocity()); 

        double speed = pidOutput + feedforwardOutput;
        
        SmartDashboard.putNumber("Shoulder_Move_Output", speed); 

        this.shoulder.setACPNormalizedVoltage(speed);
    }

    @Override
    public boolean isFinished() {
        return this.motionProfile.isFinished(this.timer.getTimeDeltaSec());
        
    }
}

package frc.robot.commands.ShintakePivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShintakePivotSubsystem;
import frc.robot.util.OrbitTimer;

public class STPGoToPositionCommand_Old extends Command {

    private ShintakePivotSubsystem ShintakePivot;
    private double angle;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State startState;
    private TrapezoidProfile.State endState;

    private OrbitTimer timer;

    public STPGoToPositionCommand_Old(ShintakePivotSubsystem ShintakePivot, double angle) {
        this.ShintakePivot = ShintakePivot;
        this.angle = angle;
        this.timer = new OrbitTimer();
        addRequirements(ShintakePivot);
    }

    @Override
    public void initialize() {
        this.ShintakePivot.movePIDController.reset();
        //this.ShintakePivot.setShintakePivotOffset(angle);

        this.ShintakePivot.setTargetAngle(angle); 

        double startVelocity = 0.0;

        if (!this.ShintakePivot.getAngularVelocity().isNaN()) {
            startVelocity = this.ShintakePivot.getAngularVelocity().doubleValue();
        }

        this.startState = new TrapezoidProfile.State(this.ShintakePivot.getShintakePivotAngle(), startVelocity);
        // this.endState = new
        // TrapezoidProfile.State(this.ShintakePivot.getTargetAngle(), 0.0);
        this.endState = new TrapezoidProfile.State(this.ShintakePivot.getTargetAngle(), 0.0);

        this.motionProfile = new TrapezoidProfile(this.ShintakePivot.ShintakePivotMotionProfileConstraints);

        this.timer.start();

        System.out.println("Set ShintakePivot Offset = " + this.angle);
    }

    @Override
    public void execute() {
        TrapezoidProfile.State profileTarget = this.motionProfile.calculate(this.timer.getTimeDeltaSec(), this.endState,
                this.startState);

        double target = profileTarget.position;
        double input = this.ShintakePivot.getShintakePivotAngle();

        SmartDashboard.putNumber("ShintakePivot_Move_Profile_Position", profileTarget.position);
        SmartDashboard.putNumber("ShintakePivot_Move_Profile_Velocity", profileTarget.velocity);

        double pidOutput = this.ShintakePivot.movePIDController.calculate(target, input);
        // SmartDashboard.putNumber("ShintakePivot_Motion_Profile_Ouput", pidOutput);

        // if (Math.abs(speed) > 0.50) speed = Math.copySign(0.5, speed);
        double feedforwardOutput = 0.0;

        if (!this.ShintakePivot.getAngularVelocity().isNaN()) {
            feedforwardOutput = this.ShintakePivot.ShintakePivotFeedForward.calculate(
                    Math.toRadians(profileTarget.position),
                    Math.toRadians(this.ShintakePivot.getAngularVelocity()));
        }

        double speed = pidOutput + feedforwardOutput;

        if (Math.abs(speed) > 0.4) {
            speed = Math.copySign(0.4, speed); // Clamping speed to prevent motor stall
        }

        // SmartDashboard.putNumber("ShintakePivot_Move_PID_Output", pidOutput);
        // SmartDashboard.putNumber("ShintakePivot_FF_Output", feedforwardOutput);
        SmartDashboard.putNumber("ShintakePivot_Move_PID_And_FF_Output", speed);
        this.ShintakePivot.updateSmartDashboard();

        this.ShintakePivot.setShintakePivotNormalizedVoltage(speed);
    }

    @Override
    public boolean isFinished() {
        return this.motionProfile.isFinished(this.timer.getTimeDeltaSec())
        /*
         * && Math.abs(this.ShintakePivot.getShintakePivotAngle() -
         * this.ShintakePivot.getTargetAngle()) < 3.0
         */;
    }
}
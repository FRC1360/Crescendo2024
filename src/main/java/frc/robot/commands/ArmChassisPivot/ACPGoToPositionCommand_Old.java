package frc.robot.commands.ArmChassisPivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmChassisPivotSubsystem;
import frc.robot.util.OrbitTimer;

public class ACPGoToPositionCommand_Old extends Command {

    private ArmChassisPivotSubsystem ACP;
    private double angle;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State startState;
    private TrapezoidProfile.State endState;

    private OrbitTimer timer;

    public ACPGoToPositionCommand_Old(ArmChassisPivotSubsystem ACP, double angle) {
        this.ACP = ACP;
        this.angle = angle;
        this.timer = new OrbitTimer();
        addRequirements(ACP);
    }

    @Override
    public void initialize() {
        this.ACP.movePIDController.reset();
        this.ACP.setTargetAngle(angle);

        System.out.println("Shoulder angle set to: " + this.ACP.getTargetAngle());

        this.startState = new TrapezoidProfile.State(this.ACP.getACPAngle(), 0.0);
        this.endState = new TrapezoidProfile.State(this.ACP.getTargetAngle(), 0.0);

        this.motionProfile = new TrapezoidProfile(this.ACP.ACPMotionProfileConstraints);

        this.timer.start();
    }

    @Override
    public void execute() {
        // Rest of your execute method remains unchanged

        TrapezoidProfile.State profileTarget;

        // prevent arm from going past vertical
        // if (this.ACP.getTargetAngle() > 90) {
        // this.ACP.setTargetAngle(90);
        // }

        profileTarget = motionProfile.calculate(this.timer.getTimeDeltaSec(),
                this.startState, this.endState);

        double target = profileTarget.position;

        SmartDashboard.putNumber("Shoulder_Move_Profile_Position", profileTarget.position);
        SmartDashboard.putNumber("Shoulder_Move_Profile_Velocity", profileTarget.velocity);

        double input = this.ACP.getACPAngle();

        double pidOutput = this.ACP.movePIDController.calculate(target, input);

        // double feedforwardOutput = this.ACP.ACPFeedForward
        // .calculate(target, this.ACP.getAngluarVelocity());

        double speed = pidOutput; // + feedforwardOutput;

        SmartDashboard.putNumber("Shoulder_Move_Output", speed);

        this.ACP.setACPNormalizedVoltage(speed);

    }

    @Override
    public boolean isFinished() {
        return this.motionProfile.isFinished(this.timer.getTimeDeltaSec());
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("Shoulder_Move_Time", this.timer.getTimeDeltaSec());
        this.ACP.setACPSpeed(0);
    }
}

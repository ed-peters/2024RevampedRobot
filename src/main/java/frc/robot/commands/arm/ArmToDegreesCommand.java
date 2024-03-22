package frc.robot.commands.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class ArmToDegreesCommand extends Command {

    public static final Constraints CONSTRAINTS = new Constraints(45.0, 45.0);
    public static final double THRESHOLD = 1.0;

    private final ArmSubsystem arm;
    private final PIDController pid;
    private final ArmFeedforward feedforward;
    private final TrapezoidProfile profile;
    private final Timer timer;
    private final State goal;

    public ArmToDegreesCommand(ArmSubsystem arm, double angle) {

        this.arm = arm;
        this.pid = arm.makePid();
        this.feedforward = arm.makeFeedforward();
        this.profile = new TrapezoidProfile(CONSTRAINTS);
        this.timer = new Timer();
        this.goal = new State(angle, 0.0);

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {

        State currentState = new State(arm.getPositionDegrees(), arm.getVelocityDps());
        State desiredState = profile.calculate(timer.get(), currentState, goal);

        double v_ff = feedforward.calculate(desiredState.position, desiredState.velocity);
        double v_pid = pid.calculate(arm.getPositionDegrees(), desiredState.position);
        arm.setVolts(v_ff + v_pid);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getPositionDegrees() - goal.position) < THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}

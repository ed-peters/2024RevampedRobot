package frc.robot.commands.beak;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.beak.FlywheelSubsystem;

/**
 * Runs a {@link FlywheelSubsystem} at a specific FPS using closed-loop control.
 */
public class FlywheelFpsCommand extends Command {

    private final FlywheelSubsystem flywheel;
    private final PIDController pid;
    private final SimpleMotorFeedforward feedforward;
    private final double setpoint;

    public FlywheelFpsCommand(FlywheelSubsystem flywheel, double setpoint) {

        this.flywheel = flywheel;
        this.pid = flywheel.getParams().makePid();
        this.feedforward = flywheel.getParams().makeFeedforward();
        this.setpoint = setpoint;

        addRequirements(flywheel);
    }

    @Override
    public void execute() {
        double v_ff = feedforward.calculate(setpoint);
        double v_pid = pid.calculate(flywheel.getWheelFps(), setpoint);
        flywheel.setVolts(v_ff + v_pid);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
    }
}

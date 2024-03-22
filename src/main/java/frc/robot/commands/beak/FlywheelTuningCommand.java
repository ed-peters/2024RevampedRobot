package frc.robot.commands.beak;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.beak.FlywheelSubsystem;
import frc.robot.util.BooleanHolder;
import frc.robot.util.Dash;
import frc.robot.util.DoubleHolder;

public class FlywheelTuningCommand extends Command {

    public static final double DEFAULT_KP = 0.0;
    public static final double DEFAULT_KV = 0.0;

    private final FlywheelSubsystem flywheel;
    private final PIDController pid;
    private final BooleanHolder enabled;
    private final DoubleHolder setpoint;
    private SimpleMotorFeedforward feedforward;

    public FlywheelTuningCommand(FlywheelSubsystem flywheel) {

        this.flywheel = flywheel;
        this.pid = flywheel.getParams().makePid();
        this.enabled = new BooleanHolder(false);
        this.setpoint = new DoubleHolder(0.0);
        this.feedforward = flywheel.getParams().makeFeedforward();

        SmartDashboard.putData(flywheel.getName()+"TuningCommand", builder -> {
            Dash.addDouble(builder, "Controls/Setpoint", setpoint);
            Dash.addBoolean(builder, "Controls/Enabled?", enabled);
            Dash.addDouble(builder, "Params/kP", pid::getP, pid::setP);
            Dash.addDouble(builder, "Params/kV", () -> feedforward.kv, val -> feedforward = new SimpleMotorFeedforward(0.0, val));
            Dash.addDouble(builder, "Output/MotorRpm", flywheel::getMotorRpm);
            Dash.addDouble(builder, "Output/WheelRpm", flywheel::getWheelRpm);
            Dash.addDouble(builder, "Output/WheelFps", flywheel::getWheelFps);
        });
    }

    @Override
    public void initialize() {
        flywheel.stop();
    }

    @Override
    public void execute() {
        if (enabled.getAsBoolean()) {
            double fps = setpoint.getAsDouble();
            double v_ff = feedforward.calculate(fps);
            double v_pid = pid.calculate(flywheel.getWheelFps(), fps);
            flywheel.setVolts(v_ff + v_pid);
        } else {
            flywheel.stop();
        }
    }
}

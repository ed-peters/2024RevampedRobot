package frc.robot.commands.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.util.BooleanHolder;
import frc.robot.util.Dash;
import frc.robot.util.DoubleHolder;

public class ArmTuningCommand extends Command {

    private final ArmSubsystem arm;
    private final PIDController pid;
    private final DoubleHolder setpointAngle;
    private final DoubleHolder setpointDps;
    private final BooleanHolder enabled;
    private ArmFeedforward feedforward;
    private double lastFeedforward;
    private double lastPid;
    private double lastVoltage;

    public ArmTuningCommand(ArmSubsystem arm) {

        this.arm = arm;
        this.pid = arm.makePid();
        this.setpointAngle = new DoubleHolder(0.0);
        this.setpointDps = new DoubleHolder(0.0);
        this.enabled = new BooleanHolder(false);
        this.feedforward = arm.makeFeedforward();

        addRequirements(arm);

        SmartDashboard.putData("ArmTuningCommand", builder -> {
            Dash.addDouble(builder, "Controls/SetpointAngle", setpointAngle);
            Dash.addDouble(builder, "Controls/SetpointDps", setpointDps);
            Dash.addBoolean(builder, "Controls/Enabled?", enabled);
            Dash.addDouble(builder, "Params/kP", pid::getP, pid::setP);
            Dash.addDouble(builder, "Params/kI", pid::getI, pid::setI);
            Dash.addDouble(builder, "Params/kD", pid::getD, pid::setD);
            Dash.addDouble(builder, "Params/kG", () -> feedforward.kg, val -> { feedforward = new ArmFeedforward(0.0, val, feedforward.kv); });
            Dash.addDouble(builder, "Params/kV", () -> feedforward.kv, val -> { feedforward = new ArmFeedforward(0.0, feedforward.kg, val); });
            Dash.addDouble(builder, "Output/Feedforward", () -> lastFeedforward);
            Dash.addDouble(builder, "Output/Pid", () -> lastPid);
            Dash.addDouble(builder, "Output/Voltage", () -> lastVoltage);
            Dash.addDouble(builder, "Output/PositionDegrees", arm::getPositionDegrees);
            Dash.addDouble(builder, "Output/VelocityDps", arm::getVelocityDps);
        });
    }

    @Override
    public void initialize() {
        stop();
    }

    @Override
    public void execute() {
        if (enabled.getAsBoolean()) {

            lastFeedforward = feedforward.calculate(setpointAngle.getAsDouble(), setpointDps.getAsDouble());
            lastPid = pid.calculate(arm.getPositionDegrees(), setpointAngle.getAsDouble());
            lastVoltage = lastFeedforward + lastPid;

            arm.setVolts(lastVoltage);

            if (Math.abs(setpointDps.getAsDouble()) > 0.0) {
                setpointAngle.accept(setpointAngle.getAsDouble() + setpointDps.getAsDouble() * 0.02);
            }

        } else {
            stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        stop();
    }

    private void stop() {
        lastFeedforward = 0.0;
        lastPid = 0.0;
        lastVoltage = 0.0;
        arm.stop();
    }
}

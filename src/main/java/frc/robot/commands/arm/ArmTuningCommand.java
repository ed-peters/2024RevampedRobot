package frc.robot.commands.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.util.BooleanHolder;
import frc.robot.util.Dash;
import frc.robot.util.DoubleHolder;

/**
 * Command for tuning the arm. To use this:
 *  - Set kG and kV to zero
 *  - Loop: pick a setpoint (e.g. 45 degrees), hold the arm there, then increase kG until the arm holds still
 *  - Loop: pick a velocity (e.g. 1 degree/sec), then increase kV until the arm tracks the movement
 *  - Set kP, kI and kD to zero
 *  - Loop:
 *      - Pick a setpoint (e.g. 45 degrees) and hold the arm there, then change the setpoint
 *      - Increase kP till it moves sharply to the new position
 *      - Increase kI if the arm gets "stuck" before hitting the new position
 *      - Increase kD to "help track a moving endpoint and limit oscillation"
 * 
 * Once you're done, copy the values of kG, kV, kP, kI and kD to {@link ArmSubsystem}
 * 
 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html
 */
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
            Dash.add(builder, "Controls/SetpointAngle", setpointAngle);
            Dash.add(builder, "Controls/SetpointDps", setpointDps);
            Dash.add(builder, "Controls/Enabled?", enabled);
            Dash.add(builder, "Params/kP", pid::getP, pid::setP);
            Dash.add(builder, "Params/kI", pid::getI, pid::setI);
            Dash.add(builder, "Params/kD", pid::getD, pid::setD);
            Dash.add(builder, "Params/kG", () -> feedforward.kg, val -> { feedforward = new ArmFeedforward(0.0, val, feedforward.kv); });
            Dash.add(builder, "Params/kV", () -> feedforward.kv, val -> { feedforward = new ArmFeedforward(0.0, feedforward.kg, val); });
            Dash.add(builder, "Output/Feedforward", () -> lastFeedforward);
            Dash.add(builder, "Output/Pid", () -> lastPid);
            Dash.add(builder, "Output/Voltage", () -> lastVoltage);
            Dash.add(builder, "Output/PositionDegrees", arm::getPositionDegrees);
            Dash.add(builder, "Output/VelocityDps", arm::getVelocityDps);
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

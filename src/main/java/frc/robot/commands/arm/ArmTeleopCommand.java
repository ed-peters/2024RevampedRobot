package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class ArmTeleopCommand extends Command {

    public static final double MAX_VOLTS_UP = 7.0;
    public static final double MAX_VOLTS_DOWN = 3.0;
    public static final double DEADBAND = 0.1;

    private final ArmSubsystem arm;
    private final DoubleSupplier percentSupplier;

    public ArmTeleopCommand(ArmSubsystem arm, DoubleSupplier percentSupplier) {

        this.arm = arm;
        this.percentSupplier = percentSupplier;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.stop();
    }

    @Override
    public void execute() {

        double pct = percentSupplier.getAsDouble();;
        pct = MathUtil.applyDeadband(pct, DEADBAND);
        pct = MathUtil.clamp(pct, -1.0, 1.0);

        if (pct == 0.0) {
            arm.stop();
        } else {
            arm.setVolts(pct * (pct < 0 ? MAX_VOLTS_DOWN : MAX_VOLTS_UP));
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}

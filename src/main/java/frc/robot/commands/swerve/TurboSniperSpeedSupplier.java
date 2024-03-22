package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.math.util.Units.*;
import static edu.wpi.first.math.MathUtil.applyDeadband;
import static edu.wpi.first.math.MathUtil.clamp;

/**
 * Implements a common control scheme:
 * - Left joystick controls translation in X and Y dimensions
 * - Right stick rotates the bot left/right
 * - Left trigger enables "sniper mode" (all translation is half speed)
 * - Right trigger enables "turbo mode" (all translation is double speed)
 */
public class TurboSniperSpeedSupplier implements Supplier<ChassisSpeeds> {

    public static final double DEFAULT_TURBO_FACTOR = 2.0;
    public static final double DEFAULT_SNIPER_FACTOR = 0.5;
    public static final double DEFAULT_MAX_DRIVE_SPEED = 7.00; // fps
    public static final double DEFAULT_MAX_TURN_SPEED = 180.0; // dps
    public static final double DEFAULT_DEADBAND = 0.1;

    private final DoubleSupplier leftX;
    private final DoubleSupplier leftY;
    private final DoubleSupplier rightX;
    private final DoubleSupplier sniperTrigger;
    private final DoubleSupplier turboTrigger;
    private double turboFactor;
    private double sniperFactor;
    private double maxDriveSpeed;
    private double maxAngleSpeed;
    private double deadband;
    private boolean forceSniperMode;

    public TurboSniperSpeedSupplier(CommandXboxController controller) {
        leftX = () -> -controller.getLeftX();
        leftY = () -> -controller.getLeftY();
        rightX = () -> -controller.getRightX();
        sniperTrigger = controller::getLeftTriggerAxis;
        turboTrigger = controller::getRightTriggerAxis;
        turboFactor = DEFAULT_TURBO_FACTOR;
        sniperFactor = DEFAULT_SNIPER_FACTOR;
        maxDriveSpeed = DEFAULT_MAX_DRIVE_SPEED;
        maxAngleSpeed = DEFAULT_MAX_TURN_SPEED;
        deadband = DEFAULT_DEADBAND;
        forceSniperMode = false;
    }

    public void forceSniperMode(boolean force) {
        forceSniperMode = force;
    }

    @Override
    public ChassisSpeeds get() {

        double jx = applyDeadband(leftY.getAsDouble(), deadband);
        double jy = applyDeadband(leftX.getAsDouble(), deadband);
        double jo = applyDeadband(rightX.getAsDouble(), deadband);

        jx = jx * Math.abs(jx);
        jy = jy * Math.abs(jy);
        jo = jo * Math.abs(jo);

        double vx = clamp(jx * maxDriveSpeed, -maxDriveSpeed, maxDriveSpeed);
        double vy = clamp(jy * maxDriveSpeed, -maxDriveSpeed, maxDriveSpeed);
        double vo = clamp(jo * maxAngleSpeed, -maxAngleSpeed, maxAngleSpeed);

        if (sniperTrigger.getAsDouble() > 0.5 || forceSniperMode) {
            vx *= sniperFactor;
            vy *= sniperFactor;
        } else if (turboTrigger.getAsDouble() > 0.5) {
            vx *= turboFactor;
            vy *= turboFactor;
        }

        return new ChassisSpeeds(feetToMeters(vx), feetToMeters(vy), degreesToRadians(vo));
    }
}

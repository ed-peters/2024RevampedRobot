package frc.robot.commands.beak;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.beak.FlywheelSubsystem;

import java.util.function.BooleanSupplier;

/**
 * Compound commands for the intake and shooter:
 *      - intake (roll intake forward until interrupted)
 *      - shoot (spin up shooter, then intake)
 *      - eject (rolls both backwards slowly)
 */
public class BeakCommands {

    // tune these for shooting
    public static final double SHOOT_FPS = 300.0;
    public static final double SHOOT_SPINUP_SECS = 0.5;
    public static final double SHOOT_SHOOT_SECS = 0.5;

    // tune this for intake
    public static final double INTAKE_FPS = 1.0;

    // tune this for eject
    public static final double EJECT_FPS = 1.0;
    public static final double EJECT_SECS = 2.0;

    public static Command intake(FlywheelSubsystem intake,
                                 FlywheelSubsystem shooter,
                                 BooleanSupplier interrupt) {

        Command roll = Commands.race(
                intake.fpsCommand(INTAKE_FPS),
                shooter.stopCommand(),
                Commands.waitUntil(interrupt));

        Command stop = Commands.parallel(
                intake.stopOnceCommand(),
                shooter.stopOnceCommand());

        return roll.andThen(stop);
    }

    public static Command shoot(FlywheelSubsystem intake, FlywheelSubsystem shooter) {

        Command spinUp = Commands.race(
                Commands.waitSeconds(SHOOT_SPINUP_SECS),
                intake.stopCommand(),
                shooter.fpsCommand(SHOOT_FPS));

        Command shoot = Commands.race(
                Commands.waitSeconds(SHOOT_SHOOT_SECS),
                intake.fpsCommand(SHOOT_FPS),
                shooter.fpsCommand(SHOOT_FPS));

        Command stop = Commands.parallel(
                intake.stopOnceCommand(),
                shooter.stopOnceCommand());

        return spinUp.andThen(shoot).andThen(stop);
    }

    public static Command eject(FlywheelSubsystem intake, FlywheelSubsystem shooter) {

        Command eject = Commands.race(
                intake.fpsCommand(-EJECT_FPS),
                shooter.fpsCommand(-EJECT_FPS),
                Commands.waitSeconds(EJECT_SECS));

        Command stop = Commands.parallel(
                intake.stopOnceCommand(),
                shooter.stopOnceCommand());

        return eject.andThen(stop);
    }
}

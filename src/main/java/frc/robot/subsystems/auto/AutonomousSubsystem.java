package frc.robot.subsystems.auto;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.beak.BeakCommands;
import frc.robot.subsystems.beak.FlywheelSubsystem;
import frc.robot.subsystems.beak.PhotoSensor;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.BooleanHolder;
import frc.robot.util.Dash;

import java.util.function.BooleanSupplier;

/**
 * Creates an autonomous program
 */
public class AutonomousSubsystem {

    private final FlywheelSubsystem intake;
    private final FlywheelSubsystem shooter;
    private final SwerveDriveSubsystem drive;
    private final PhotoSensor sensor;
    private final BooleanHolder red;

    public AutonomousSubsystem(RobotContainer container) {

        intake = container.intake;
        shooter = container.shooter;
        drive = container.drive;
        sensor = container.photoSensor;
        red = new BooleanHolder(false);

        SmartDashboard.putData("AutonomousSubsystem", builder -> {
            Dash.add(builder, "Red?", red);
        });
    }

    public Command getAutonomousCommand() {

        AutonomousPath middle = new AutonomousPath(drive, "QuadShotMiddle", red.getAsBoolean());
        AutonomousPath south = new AutonomousPath(drive, "QuadShotSouth", red.getAsBoolean());
        AutonomousPath north = new AutonomousPath(drive, "QuadShotNorth", red.getAsBoolean());
        north.useForInitialPose();

        return Commands.sequence(
                shoot(),
                Commands.parallel(north, intake()),
                shoot(),
                middle,
                shoot(),
                south);
    }

    private Command intake() {
        BooleanSupplier interrupt = RobotBase.isSimulation() ? () -> true : sensor;
        return BeakCommands.intake(intake, shooter, interrupt);
    }

    private Command shoot() {
        return Commands.race(
                BeakCommands.shoot(intake, shooter),
                drive.stopCommand());
    }
}

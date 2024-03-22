package frc.robot.subsystems.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.beak.BeakCommands;
import frc.robot.subsystems.beak.FlywheelSubsystem;
import frc.robot.subsystems.beak.PhotoSensor;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.Dash;

import java.util.Objects;
import java.util.function.BooleanSupplier;

public class AutonomousSubsystem {

    private final FlywheelSubsystem intake;
    private final FlywheelSubsystem shooter;
    private final SwerveDriveSubsystem drive;
    private final PhotoSensor sensor;

    public AutonomousSubsystem(RobotContainer container) {

        intake = container.intake;
        shooter = container.shooter;
        drive = container.drive;
        sensor = container.photoSensor;

        SmartDashboard.putData("AutonomousSubsystem", builder -> {
            Dash.add(builder, "Alliance", () -> Objects.toString(DriverStation.getAlliance()));
        });
    }

    private boolean isRedAlliance() {
        // Alliance alliance = DriverStation.getAlliance().orElse(null);
        // return alliance == Alliance.Red;
        return false;
    }

    public Command getAutonomousCommand() {

        AutonomousPath north = new AutonomousPath(drive, "QuadShotNorth", isRedAlliance());
//        AutonomousPath middle = new AutonomousPath(drive, "QuadShotMiddle", isRedAlliance());
//        AutonomousPath south = new AutonomousPath(drive, "QuadShotSouth", isRedAlliance());

        return Commands.sequence(
                shoot(),
                Commands.parallel(north, intake()),
                shoot()
                // Commands.parallel(drive(middle), intake()),
                // shoot(),
                // Commands.parallel(drive(south), intake()),
                // shoot());
        );
    }

    private Command intake() {
        BooleanSupplier interrupt = RobotBase.isSimulation() ? () -> true : sensor;
        return BeakCommands.intake(intake, shooter, interrupt);
    }

    private Command shoot() {
        return BeakCommands.shoot(intake, shooter);
    }
}

package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.BooleanHolder;
import frc.robot.util.Dash;

import java.util.function.Supplier;

/**
 * Simple teleop for swerve mode
 */
public class SwerveTeleopCommand extends Command {

    private final SwerveDriveSubsystem drive;
    private final Supplier<ChassisSpeeds> speedSupplier;
    private final BooleanHolder fieldRelative;
    private ChassisSpeeds lastSpeeds;

    public SwerveTeleopCommand(SwerveDriveSubsystem drive, Supplier<ChassisSpeeds> speedSupplier) {

        this.drive = drive;
        this.speedSupplier = speedSupplier;
        this.fieldRelative = new BooleanHolder(true);
        this.lastSpeeds = new ChassisSpeeds();

        addRequirements(drive);

        SmartDashboard.putData("SwerveTeleopCommand", builder -> {
            Dash.add(builder, "FieldRelative?", fieldRelative);
            Dash.add(builder, "Speeds/X", () -> lastSpeeds.vxMetersPerSecond);
            Dash.add(builder, "Speeds/Y", () -> lastSpeeds.vyMetersPerSecond);
            Dash.add(builder, "Speeds/Omega", () -> Units.radiansToDegrees(lastSpeeds.omegaRadiansPerSecond));
        });
    }

    @Override
    public void initialize() {
        drive.stop();
    }

    @Override
    public void execute() {
        ChassisSpeeds nextSpeeds = speedSupplier.get();
        if (fieldRelative.getAsBoolean()) {
            drive.driveFieldRelative(nextSpeeds);
        } else {
            drive.driveRobotRelative(nextSpeeds);
        }
        lastSpeeds = nextSpeeds;
    }

    public void end(boolean interrupted) {
        drive.stop();
    }
}

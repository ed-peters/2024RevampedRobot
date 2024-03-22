package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;

import frc.robot.util.Dash;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveDriveSubsystem extends SubsystemBase {

    public static ChassisSpeeds STOP = new ChassisSpeeds();

    // Maximum speed of the robot, used for both translation and rotation
    public static final double MAX_SPEED = Units.feetToMeters(14.5);

    // Angle conversion factor is 360 / GEAR RATIO
    // Drive conversion factor is (PI * WHEEL DIAMETER IN METERS) / GEAR RATIO
    // SDS module specs here: https://www.andymark.com/products/mk4i-swerve-modules
    public static final double ANGLE_CONVERSION = SwerveMath.calculateDegreesPerSteeringRotation(150.0 / 7.0);
    public static final double DRIVE_CONVERSION = SwerveMath.calculateMetersPerRotation(
            Units.inchesToMeters(3.78),
            6.75);

    private final SwerveDrive drive;
    private int poseResetCount;

    public SwerveDriveSubsystem() {

        File dir = new File(Filesystem.getDeployDirectory(), "swerve");

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;
        try {
            drive = new SwerveParser(dir).createSwerveDrive(MAX_SPEED);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        // Heading correction only used while controlling the robot via angle
        drive.setHeadingCorrection(false);

        // Cosine compensation disabled for simulations since it causes discrepancies not seen in real life
        drive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);

        SmartDashboard.putData("SwerveDriveSubsystem", builder -> {
            Dash.addPose(builder, "Pose/", this::getPose);
            Dash.add(builder, "ResetCount", () -> poseResetCount);
        });
    }

    // ================================================================
    // GYRO
    // ================================================================

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public Command zeroGyroCommand() {
        return runOnce(drive::zeroGyro);
    }

    // ================================================================
    // POSE AND ODOMETRY
    // ================================================================

    public Pose2d getPose() {
        return drive.getPose();
    }

    public void resetOdometry(Pose2d pose) {
        drive.resetOdometry(pose);
        poseResetCount += 1;
    }

    // ================================================================
    // KINEMATICS & DRIVING
    // ================================================================

    public SwerveDriveKinematics getKinematics() {
        return drive.kinematics;
    }

    public ChassisSpeeds getFieldRelativeVelocity() {
        return drive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotRelativeVelocity() {
        return drive.getRobotVelocity();
    }

    public void stop() {
        setChassisSpeeds(STOP);
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        drive.driveFieldOriented(speeds);
    }

    public void driveRobotRelative(ChassisSpeeds velocity) {
        drive.drive(velocity);
    }

    // set raw speeds (skips discretization, heading correction etc.)
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        drive.setChassisSpeeds(chassisSpeeds);
    }

    public void postTrajectory(Trajectory trajectory) {
        drive.postTrajectory(trajectory);
    }

    // ================================================================
    // BRAKES
    // ================================================================

    public void setBrakes(boolean brakes) {
        drive.setMotorIdleMode(brakes);
    }

    public void lockWheels() {
        drive.lockPose();
    }
}

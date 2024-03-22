package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase
{
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

    public SwerveSubsystem(File directory) {

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            drive = new SwerveParser(directory).createSwerveDrive(MAX_SPEED);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        // Heading correction only used while controlling the robot via angle
        drive.setHeadingCorrection(false);

        // Cosine compensation disabled for simulations since it causes discrepancies not seen in real life
        drive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
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

    /**
     * Command to drive the robot using translative values and heading as a setpoint.
     *
     * @param translationX Translation in the X direction. Cubed for smoother controls.
     * @param translationY Translation in the Y direction. Cubed for smoother controls.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                                DoubleSupplier headingY)
    {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
        return run(() -> {
            double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
            double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumVelocity()));
        });
    }

    /**
     * Command to drive the robot using translative values and heading as a setpoint.
     *
     * @param translationX Translation in the X direction.
     * @param translationY Translation in the Y direction.
     * @param rotation     Rotation as a value between [-1, 1] converted to radians.
     * @return Drive command.
     */
    public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation)
    {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
        return run(() -> {
            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
                    translationY.getAsDouble(),
                    rotation.getAsDouble() * Math.PI,
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumVelocity()));
        });
    }

    /**
     * Command to drive the robot using translative values and heading as angular velocity.
     *
     * @param translationX     Translation in the X direction. Cubed for smoother controls.
     * @param translationY     Translation in the Y direction. Cubed for smoother controls.
     * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
    {
        return run(() -> {
            // Make the robot move
            swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
                            Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()),
                    Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
                    true,
                    false);
        });
    }

}

package frc.robot.subsystems.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class AutonomousPath extends Command {

    public static final double KP_TRANS = 1.0;
    public static final double KP_ROT = 1.0;
    public static final double [] STOP = { 0.0, 0.0, 0.0 };

    private final SwerveDriveSubsystem drive;
    private final ChoreoTrajectory trajectory;
    private final ChoreoControlFunction control;
    private final boolean isMirrored;
    private final Timer timer;
    private double [] desiredPose;
    private double [] desiredVelocity;
    private double [] desiredSpeeds;
    private boolean running;

    public AutonomousPath(SwerveDriveSubsystem drive, String name, boolean isMirrored) {

        this.drive = drive;
        this.trajectory = Choreo.getTrajectory(name);
        this.control = makeControlFunction();
        this.isMirrored = isMirrored;
        this.timer = new Timer();
        this.running = false;

        clearMetrics();

        addRequirements(drive);

        SmartDashboard.putData(name, builder -> {
            builder.addDoubleArrayProperty("desiredPose", () -> desiredPose, null);
            builder.addDoubleArrayProperty("desiredVelocity", () -> desiredVelocity, null);
            builder.addDoubleArrayProperty("desiredSpeeds", () -> desiredSpeeds, null);
            builder.addDoubleProperty("running", () -> running ? 1.0 : 0.0, null);
        });
    }

    public Pose2d getInitialPose() {
        return trajectory.getInitialPose();
    }

    private ChoreoControlFunction makeControlFunction() {

        PIDController px = new PIDController(KP_TRANS, 0.0, 0.0);
        PIDController py = new PIDController(KP_TRANS, 0.0, 0.0);
        PIDController po = new PIDController(KP_ROT, 0.0, 0.0);
        po.enableContinuousInput(-Math.PI, Math.PI);

        return (pose, referenceState) -> {
            double xFF = referenceState.velocityX;
            double yFF = referenceState.velocityY;
            double oFF = referenceState.angularVelocity;
            double xFeedback = px.calculate(pose.getX(), referenceState.x);
            double yFeedback = py.calculate(pose.getY(), referenceState.y);
            double oFeedback = po.calculate(pose.getRotation().getRadians(), referenceState.heading);
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    xFF + xFeedback,
                    yFF + yFeedback,
                    oFF + oFeedback, pose.getRotation());
        };
    }

    @Override
    public void initialize() {
        timer.restart();
        clearMetrics();
        running = true;
    }

    @Override
    public void execute() {

        Pose2d currentPose = drive.getPose();
        ChoreoTrajectoryState desiredState = trajectory.sample(timer.get(), isMirrored);
        ChassisSpeeds speeds = control.apply(currentPose, desiredState);

        desiredPose = new double[]{ desiredState.x, desiredState.y, Units.radiansToDegrees(desiredState.heading) };
        desiredVelocity = new double[]{ desiredState.velocityX, desiredState.velocityY, Units.radiansToDegrees(desiredState.angularVelocity) };
        desiredSpeeds = new double[]{ speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, Units.radiansToDegrees(speeds.omegaRadiansPerSecond) };

        drive.setChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTime());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drive.stop();
        clearMetrics();
        running = false;
    }

    private void clearMetrics() {
        desiredPose = STOP;
        desiredVelocity = STOP;
        desiredSpeeds = STOP;
    }
}
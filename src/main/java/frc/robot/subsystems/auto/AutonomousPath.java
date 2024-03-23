package frc.robot.subsystems.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class AutonomousPath extends Command {

    public static final double KP_TRANS = 1.0;
    public static final double KP_ROT = 1.0;

    private final SwerveDriveSubsystem drive;
    private final ChoreoTrajectory trajectory;
    private final ChoreoControlFunction control;
    private final boolean isMirrored;
    private final Timer timer;
    private boolean setInitialPose;

    public AutonomousPath(SwerveDriveSubsystem drive, String name, boolean isMirrored) {

        this.drive = drive;
        this.trajectory = Choreo.getTrajectory(name);
        this.control = makeControlFunction();
        this.isMirrored = isMirrored;
        this.timer = new Timer();
        this.setInitialPose = false;

        addRequirements(drive);
    }

    public Pose2d getInitialPose() {
        return trajectory.getInitialPose();
    }

    public void useForInitialPose() {
        setInitialPose = true;
    }

    @SuppressWarnings("resource")
    private ChoreoControlFunction makeControlFunction() {

        PIDController px = new PIDController(KP_TRANS, 0.0, 0.0);
        PIDController py = new PIDController(KP_TRANS, 0.0, 0.0);
        PIDController po = new PIDController(KP_ROT, 0.0, 0.0);
        po.enableContinuousInput(-Math.PI, Math.PI);

        return (currentPose, desiredState) -> {
            double xFF = desiredState.velocityX;
            double yFF = desiredState.velocityY;
            double oFF = desiredState.angularVelocity;
            double xFeedback = px.calculate(currentPose.getX(), desiredState.x);
            double yFeedback = py.calculate(currentPose.getY(), desiredState.y);
            double oFeedback = po.calculate(currentPose.getRotation().getRadians(), desiredState.heading);
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    xFF + xFeedback,
                    yFF + yFeedback,
                    oFF + oFeedback, currentPose.getRotation());
        };
    }

    @Override
    public void initialize() {

        timer.restart();

        if (setInitialPose) {
            drive.resetOdometry(trajectory.getInitialPose());
        }
    }

    @Override
    public void execute() {

        Pose2d currentPose = drive.getPose();
        ChoreoTrajectoryState desiredState = trajectory.sample(timer.get(), isMirrored);
        ChassisSpeeds speeds = control.apply(currentPose, desiredState);

        SmartDashboard.putNumberArray("Desired/Pose", new double[]{ desiredState.x, desiredState.y });
        SmartDashboard.putNumberArray("Desired/Velocity", new double[]{ desiredState.velocityX, desiredState.velocityY });
        SmartDashboard.putNumberArray("Desired/Speeds", new double[]{ speeds.vxMetersPerSecond, speeds.vyMetersPerSecond });

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
    }
}

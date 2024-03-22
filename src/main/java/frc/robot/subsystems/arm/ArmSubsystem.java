package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Dash;
import frc.robot.util.Motor;

public class ArmSubsystem extends SubsystemBase {

    public static final double DEFAULT_KP = 0.0;
    public static final double DEFAULT_KI = 0.0;
    public static final double DEFAULT_KD = 0.0;
    public static final double DEFAULT_KG = 0.0;
    public static final double DEFAULT_KV = 0.0;

    public static final double DEGREES_PER_TURN = 360.0 * (1.0 / (75.0 / (24.0 / 64.0)));

    private final CANSparkMax followMotor;
    private final CANSparkMax leadMotor;
    private final RelativeEncoder leadEncoder;

    private double lastVoltage;
    private double lastAngle;
    private double lastVelocity;

    public ArmSubsystem() {

        followMotor = Motor.getMotor("armFollower");
        leadMotor = Motor.getMotor("armLeader");
        leadEncoder = leadMotor.getEncoder();

        SmartDashboard.putData("ArmSubsystem", builder -> {
            Dash.add(builder, "Leader/", leadMotor);
            Dash.add(builder, "Follower/", followMotor);
            Dash.add(builder, "PositionDegrees", this::getPositionDegrees);
            Dash.add(builder, "VelocityDps", this::getVelocityDps);
            Dash.add(builder, "VelocityDps", this::getVelocityDps);
            Dash.add(builder, "LastVoltage", () -> lastVoltage);
            Dash.addBrake(builder, "BrakeEnabled?", leadMotor, followMotor);
        });
    }

    public double getPositionDegrees() {
        return leadEncoder.getPosition() * DEGREES_PER_TURN;
    }

    public double getVelocityDps() {
        return lastVelocity;
    }

    public PIDController makePid() {
        return new PIDController(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    }

    public ArmFeedforward makeFeedforward() {
        return new ArmFeedforward(0.0, DEFAULT_KG, DEFAULT_KV);
    }

    public void stop() {
        setVolts(0.0);
    }

    public void setVolts(double volts) {
        if (volts == 0.0) {
            lastVoltage = 0.0;
            leadMotor.stopMotor();
        } else {
            lastVoltage = MathUtil.clamp(volts, -12.0, 12.0);
            leadMotor.setVoltage(lastVoltage);
        }
    }

    @Override
    public void periodic() {
        double currentAngle = getPositionDegrees();
        double angleDelta = MathUtil.applyDeadband(currentAngle - lastAngle, 1e-2);
        lastVelocity = angleDelta / 0.02;
        lastAngle = currentAngle;
    }
}

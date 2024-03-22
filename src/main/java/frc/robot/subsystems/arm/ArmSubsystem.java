package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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

    public static final double GEAR_RATIO = 1.0 / (75.0 / (24.0 / 64.0));
    public static final double DEGREES_PER_TURN = 360.0 * GEAR_RATIO;

    private final CANSparkMax followMotor;
    private final CANSparkMax leadMotor;
    private final RelativeEncoder leadEncoder;
    private final SingleJointedArmSim sim;

    private double lastVoltage;
    private double lastAngle;
    private double lastVelocity;

    public ArmSubsystem() {

        if (RobotBase.isSimulation()) {
            followMotor = null;
            leadMotor = null;
            leadEncoder = null;
            sim = makeSim();
        } else {
            followMotor = Motor.getMotor("armFollower");
            leadMotor = Motor.getMotor("armLeader");
            leadEncoder = leadMotor.getEncoder();
            sim = null;
        }

        SmartDashboard.putData("ArmSubsystem", builder -> {
            if (leadMotor != null) {
                Dash.add(builder, "Leader/", leadMotor);
                Dash.add(builder, "Follower/", followMotor);
                Dash.addBrake(builder, "BrakeEnabled?", leadMotor, followMotor);
            } else {
                Dash.add(builder, "SimAmps", sim::getCurrentDrawAmps);
                Dash.add(builder, "SimMax?", sim::hasHitUpperLimit);
                Dash.add(builder, "SimMin?", sim::hasHitLowerLimit);
            }
            Dash.add(builder, "PositionDegrees", this::getPositionDegrees);
            Dash.add(builder, "VelocityDps", this::getVelocityDps);
            Dash.add(builder, "VelocityDps", this::getVelocityDps);
            Dash.add(builder, "LastVoltage", () -> lastVoltage);
        });
    }

    public double getPositionDegrees() {
        if (leadEncoder != null) {
            return leadEncoder.getPosition() * DEGREES_PER_TURN;
        } else {
            return Units.radiansToDegrees(sim.getAngleRads());
        }
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

        lastVoltage = MathUtil.clamp(volts, -12.0, 12.0);

        if (leadMotor != null) {
            if (lastVoltage == 0.0) {
                leadMotor.stopMotor();
            } else {
                leadMotor.setVoltage(lastVoltage);
            }
        } else {
            sim.setInputVoltage(lastVoltage);
            if (lastVoltage == 0.0) {
                sim.setState(sim.getAngleRads(), 0.0);
            }
        }
    }

    @Override
    public void periodic() {
        if (leadMotor != null) {
            double currentAngle = getPositionDegrees();
            double angleDelta = MathUtil.applyDeadband(currentAngle - lastAngle, 1e-2);
            lastVelocity = angleDelta / 0.02;
            lastAngle = currentAngle;
        }
    }

    @Override
    public void simulationPeriodic() {
        sim.update(0.02);
        lastAngle = Units.radiansToDegrees(sim.getAngleRads());
        lastVelocity = Units.radiansToDegrees(sim.getVelocityRadPerSec());
    }

    public static SingleJointedArmSim makeSim() {

        double weightKg = 25.0 / 2.205;
        double lengthM = Units.feetToMeters(3.0);
        DCMotor motor = DCMotor.getNEO(2);

        return new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(
                        motor,
                        weightKg * lengthM * lengthM,
                        1.0 / GEAR_RATIO),
                motor,
                1.0 / GEAR_RATIO,
                lengthM,
                0.0,
                Units.degreesToRadians(90.0),
                true,
                0.0);
    }
}

package frc.robot.subsystems.beak;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.beak.FlywheelFpsCommand;
import frc.robot.util.Dash;
import frc.robot.util.Motor;

public class FlywheelSubsystem extends SubsystemBase {

    private final FlywheelParams params;
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final FlywheelSim sim;
    private double lastVoltage;

    public FlywheelSubsystem(FlywheelParams params) {

        this.params = params;
        if (RobotBase.isSimulation()) {
            this.motor = null;
            this.encoder = null;
            this.sim = params.makeSim();
        } else {
            this.motor = Motor.getMotor(params.name);
            this.encoder = motor.getEncoder();
            this.sim = null;
        }

        SmartDashboard.putData(params.getSubsystemName(), builder -> {
            if (motor != null) {
                Dash.addMotor(builder, motor);
                Dash.addBrake(builder, "BrakeEnabled?", motor);
            }
            Dash.addDouble(builder, "WheelRpm", this::getWheelRpm);
            Dash.addDouble(builder, "WheelFps", this::getWheelFps);
            Dash.addDouble(builder, "LastVoltage", () -> lastVoltage);
        });
    }

    public String getName() {
        return params.getSubsystemName();
    }

    public FlywheelParams getParams() {
        return params;
    }

    public double getMotorRpm() {
        if (encoder != null) {
            return encoder.getVelocity();
        } else {
            return sim.getAngularVelocityRPM() / params.gearRatio;
        }
    }

    public double getWheelRpm() {
        return getMotorRpm() * params.gearRatio;
    }

    public double getWheelFps() {
        return getWheelRpm() * params.circumference / 60.0;
    }

    public void stop() {
        setVolts(0.0);
    }

    public void setVolts(double volts) {
        if (volts == 0.0) {
            lastVoltage = 0.0;
            if (motor != null) {
                motor.stopMotor();
            }
        } else {
            lastVoltage = MathUtil.clamp(volts, -12.0, 12.0);
            if (motor != null) {
                motor.setVoltage(lastVoltage);
            }
        }
        if (sim != null) {
            if (lastVoltage == 0.0) {
                sim.setState(0.0);
            }
            sim.setInputVoltage(lastVoltage);
        }
    }

    @Override
    public void periodic() {
        if (sim != null) {
            sim.update(0.02);
        }
    }

    public Command stopCommand() {
        return run(this::stop);
    }

    public Command stopOnceCommand() {
        return runOnce(this::stop);
    }

    public Command fpsCommand(double fps) {
        return new FlywheelFpsCommand(this, fps);
    }
}

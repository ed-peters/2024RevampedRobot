package frc.robot.subsystems.beak;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelParams {

    public final String name;
    public final double kP;
    public final double kV;
    public final double gearRatio;
    public final double circumference;

    public FlywheelParams(String name, double kP, double kV, double gearRatio, double circumference) {
        this.name = name;
        this.kP = kP;
        this.kV = kV;
        this.gearRatio = gearRatio;
        this.circumference = circumference;
    }

    public String getSubsystemName() {
        StringBuilder builder = new StringBuilder();
        builder.append(Character.toUpperCase(name.charAt(0)));
        builder.append(name.substring(1));
        builder.append("Subsystem");
        return builder.toString();
    }

    public PIDController makePid() {
        return new PIDController(kP, 0.0, 0.0);
    }

    public SimpleMotorFeedforward makeFeedforward() {
        return new SimpleMotorFeedforward(0.0, kV);
    }

    public FlywheelSim makeSim() {
        return new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(kV, 0.001),
                DCMotor.getNEO(1),
                gearRatio);
    }
}

package frc.robot.util;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import static com.revrobotics.CANSparkBase.IdleMode;

public class Motor {

    public static final Map<String,MotorDefaults> DEFAULTS;
    static {
        ObjectMapper mapper = new ObjectMapper();
        File file = new File(Filesystem.getDeployDirectory(), "motors.json");
        TypeReference<HashMap<String,MotorDefaults>> typeRef = new TypeReference<>() {};
        try {
            DEFAULTS = mapper.readValue(new JsonFactory().createParser(file), typeRef);
        } catch (IOException e) {
            throw new RuntimeException("error reading motor defaults", e);
        }
    }

    public static final Map<String,CANSparkMax> MOTORS = new HashMap<>();

    public static CANSparkMax getMotor(String name) {
        return getMotor(name, -1);
    }

    public static CANSparkMax getMotor(String name, int overrideCanId) {

        CANSparkMax motor = MOTORS.get(name);
        if (motor != null) {
            return motor;
        }

        MotorDefaults defaults = DEFAULTS.get(name);
        if (defaults == null) {
            throw new IllegalArgumentException("unknown motor " + name + "; options are " + DEFAULTS.keySet());
        }

        motor = new CANSparkMax(overrideCanId < 0 ? defaults.canId : overrideCanId, CANSparkMax.MotorType.kBrushless);
        motor.setIdleMode(defaults.brakeEnabled ? IdleMode.kBrake : IdleMode.kCoast);
        motor.setSmartCurrentLimit(defaults.currentLimit);
        motor.setOpenLoopRampRate(defaults.openLoopRampRate);
        motor.setClosedLoopRampRate(defaults.closedLoopRampRate);
        motor.setInverted(defaults.inverted);

        SparkPIDController pid = motor.getPIDController();
        pid.setP(defaults.kP);
        pid.setI(defaults.kI);
        pid.setD(defaults.kD);

        if (defaults.leader != null) {
            CANSparkMax leader = getMotor(defaults.leader);
            motor.follow(leader, defaults.followInverted);
        }

        motor.burnFlash();

        MOTORS.put(name, motor);
        return motor;
    }

    public static class MotorDefaults {

        public int canId = -1;
        public boolean inverted = false;
        public boolean brakeEnabled = false;
        public double openLoopRampRate = 0.1;
        public double closedLoopRampRate = 0.1;
        public int currentLimit = 25;
        public double kP = 0.0;
        public double kI = 0.0;
        public double kD = 0.0;
        public String leader = null;
        public boolean followInverted = false;
    }
}

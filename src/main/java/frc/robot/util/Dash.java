package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import static com.revrobotics.CANSparkBase.IdleMode;

public class Dash {

    public static void add(SendableBuilder builder, String key, DoubleSupplier getter) {
        builder.addDoubleProperty(key, getter, null);
    }

    public static void add(SendableBuilder builder, String key, DoubleSupplier getter, DoubleConsumer setter) {
        builder.addDoubleProperty(key, getter, setter);
    }

    public static void add(SendableBuilder builder, String key, BooleanHolder holder) {
        builder.addBooleanProperty(key, holder, holder);
    }

    public static void add(SendableBuilder builder, String key, BooleanSupplier getter) {
        builder.addBooleanProperty(key, getter, null);
    }

    public static void add(SendableBuilder builder, String key, DoubleHolder holder) {
        builder.addDoubleProperty(key, holder, holder);
    }

    public static void add(SendableBuilder builder, CANSparkMax motor) {
        add(builder, "", motor);
    }

    public static void add(SendableBuilder builder, String prefix, CANSparkMax motor) {
        RelativeEncoder encoder = motor.getEncoder();
        builder.addDoubleProperty(prefix+"Amps", motor::getOutputCurrent, null);
        builder.addDoubleProperty(prefix+"Volts", () -> motor.getAppliedOutput() * motor.getBusVoltage(), null);
        builder.addDoubleProperty(prefix+"Position", encoder::getPosition, null);
        builder.addDoubleProperty(prefix+"Rpm", encoder::getVelocity, null);
    }

    public static void addBrake(SendableBuilder builder, String key, CANSparkMax... motors) {
        MotorBrake brake = new MotorBrake(motors);
        builder.addBooleanProperty(key, brake, brake);
    }

    public static class MotorBrake implements BooleanSupplier, BooleanConsumer {

        private final CANSparkMax [] motors;
        private boolean enabled;

        public MotorBrake(CANSparkMax... motors) {
            this.motors = motors;
            this.enabled = motors[0].getIdleMode() == IdleMode.kBrake;
        }

        @Override
        public boolean getAsBoolean() {
            return enabled;
        }

        @Override
        public void accept(boolean value) {
            if (value != enabled) {
                for (CANSparkMax motor : motors) {
                    motor.setIdleMode(value ? IdleMode.kBrake : IdleMode.kCoast);
                }
                enabled = value;
            }
        }
    }
}

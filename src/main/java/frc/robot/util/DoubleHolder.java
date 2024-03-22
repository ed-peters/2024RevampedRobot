package frc.robot.util;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class DoubleHolder implements DoubleSupplier, DoubleConsumer {

    private double value;

    public DoubleHolder(double initial) {
        value = initial;
    }

    @Override
    public void accept(double value) {
        this.value = value;
    }

    @Override
    public double getAsDouble() {
        return value;
    }
}

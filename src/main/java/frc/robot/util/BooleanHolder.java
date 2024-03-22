package frc.robot.util;

import edu.wpi.first.util.function.BooleanConsumer;

import java.util.function.BooleanSupplier;

public class BooleanHolder implements BooleanSupplier, BooleanConsumer {

    private boolean value;

    public BooleanHolder(boolean initial) {
        value = initial;
    }

    @Override
    public void accept(boolean value) {
        this.value = value;
    }

    @Override
    public boolean getAsBoolean() {
        return value;
    }
}

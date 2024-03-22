package frc.robot.subsystems.beak;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.BooleanHolder;
import frc.robot.util.Dash;

public class PhotoSensor implements BooleanSupplier {

    public static final int CHANNEL_ID = 1;

    private final BooleanSupplier supplier;

    public PhotoSensor() {
        if (RobotBase.isSimulation()) {
            BooleanHolder sim = new BooleanHolder(false);
            SmartDashboard.putData("PhotoSensor", builder -> {
                Dash.add(builder, "Sensor", sim);
            });
            supplier = sim;
        } else {
            supplier = () -> !new DigitalInput(CHANNEL_ID).get();
        }
    }

    public boolean getAsBoolean() {
        return supplier.getAsBoolean();
    }
}

import frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Amps;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.Logger;

public interface KickerIO {

    public static class KickerIOInputs {
        public double kickerSpeed = 0.0;
    }

    public default void updateInputs(KickerIOInputs inputs) {}

    public default void setKickerSpeed(double speed) {}

    public default void stop() {}
}
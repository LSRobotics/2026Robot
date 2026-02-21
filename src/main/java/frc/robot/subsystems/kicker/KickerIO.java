package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.Logger;

public interface KickerIO {

    @AutoLog
    public static class KickerIOInputs {
        public AngularVelocity kickerSpeed = DegreesPerSecond.of(0);
    }

    public default void updateInputs(KickerIOInputs inputs) {}

    public default void setKickerSpeed(double speed) {}

    public default void stop() {}
}
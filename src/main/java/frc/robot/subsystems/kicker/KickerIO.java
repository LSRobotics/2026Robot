package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;
import static org.wpilib.units.Units.RPM;
import org.wpilib.units.measure.AngularVelocity;
import static org.wpilib.units.Units.Volts;
import org.wpilib.units.measure.Voltage;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;

import org.wpilib.units.measure.Current;
import org.littletonrobotics.junction.Logger;

public interface KickerIO {

    @AutoLog
    public static class KickerIOInputs {
        public AngularVelocity kickerVelocity = DegreesPerSecond.of(0);
        public double kickerSpeed = 0;
    }

    public default void updateInputs(KickerIOInputs inputs) {}

    public default void setKickerSpeed(double speed) {}

    public default void stop() {}
}
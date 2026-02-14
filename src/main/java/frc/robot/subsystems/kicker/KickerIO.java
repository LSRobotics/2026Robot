package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Volts;

public interface KickerIO {
    @AutoLog
    public static class KickerIOInputs {
        public Current kickerMotorCurrent = Amps.of(0);
        public AngularVelocity kickerSpeed = RPM.of(0);
    }

    public default void updateInputs(KickerIOInputs inputs) {}

    public default void setKickerSpeed(double speed) {}

    public default void setVoltage(Voltage voltage) {}

    public default void stop() {}
}

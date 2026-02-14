package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Volts;

public interface SpindexerIO {
    
     @AutoLog
    public static class SpindexerIOInputs {
        public Current spindexerMotorCurrent = Amps.of(0);
        public AngularVelocity spindexerSpeed = RPM.of(0);
    }

    public default void updateInputs(SpindexerIOInputs inputs) {}

    public default void setSpindexerSpeed(double speed) {}

    public default void setVoltage(Voltage voltage) {}

    public default void stop() {}

}

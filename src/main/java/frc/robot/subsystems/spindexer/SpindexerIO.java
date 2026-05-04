package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RPM;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;
import static org.wpilib.units.Units.Volts;

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

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface ShooterFlywheelIO {
    @AutoLog
    public class ShooterFlywheelIOInputs {
        public AngularVelocity velocity = RPM.of(0);
        public Current motor1Current = Amps.of(0);
        public Current motor2Current = Amps.of(0);
    }

    public default void setVelocity(AngularVelocity velocity) {}

    public default void setVoltage(Voltage voltage) {}

    public default void setSpeed(double speed) {}

    public default void stop() {}
    
} 
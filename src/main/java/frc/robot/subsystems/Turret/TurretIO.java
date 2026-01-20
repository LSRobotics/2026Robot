package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface TurretIO {

    @AutoLog
    public static class TurretIOInputs {
        public Angle turretAngle = Degrees.of(0);
        public AngularVelocity turretVelocity = DegreesPerSecond.of(0);
        public boolean atForwardLimit = false;
        public boolean atReverseLimit = false;
        public Current turretCurrent = Amps.of(0);
        public AngularVelocity shooterSpeed = RotationsPerSecond.of(0);
    }

    public default void updateInputs(TurretIOInputs inputs){};

    public default void setTurretVoltage(Voltage voltage){};

    public default void setTurretSpeed(double speed){};

    public default void resetTurretEncoder(Angle angle){};

    
}
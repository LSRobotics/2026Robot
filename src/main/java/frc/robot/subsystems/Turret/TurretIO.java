package frc.robot.subsystems.Turret;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public interface TurretIO {

    @AutoLog
    public static class TurretIOInputs {
        public Angle motorAngle = Degrees.of(0);
        public AngularVelocity turretVelocity = DegreesPerSecond.of(0);
        public Angle turretAngle = Degrees.of(0);
        public boolean atForwardLimit = false;
        public boolean atReverseLimit = false;
        public Current turretCurrent = Amps.of(0);
        public double turretDegrees = 0d;
    }

    public default void updateInputs(TurretIOInputs inputs){};

    public default void setTurretVoltage(Voltage voltage){};

    public default void setTurretVoltage(double voltage){};


    public default void setTurretSpeed(double speed){};

    public default void resetTurretEncoder(Angle angle){};

    public default void zeroEncoder(){};

    
}
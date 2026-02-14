package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class ShooterConstants {
    public class FlywheelConstants {
        public static final AngularVelocity maxSpeed = RPM.of(-1);// TODO
        public static final AngularVelocity holdSpeed = RPM.of(10); // TODO
        public static final int flywheelMotor1ID = 63;
        public static final Current statorCurrentLimit = Amps.of(40);
        public static final int flywheelMotor2ID = 64;
        public static final double kS = 0.1; // TODO
        public static final double kV = 0.1; // TODO
        public static final Voltage maxVoltage = Volts.of(5);
        public static final double gearRatio = 1d/1d;
    }

    public class HoodConstants {
        public static final int hoodLinearActuatorPWMID = 0;
        public static final Distance actuatorLength = Millimeters.of(100);
    }
}

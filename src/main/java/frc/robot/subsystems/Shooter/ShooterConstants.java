package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class ShooterConstants {
    public static class FlywheelConstants {
        public static final AngularVelocity maxSpeed = RPM.of(100);// TODO
        public static final AngularVelocity holdSpeed = RPM.of(1); // TODO
        public static final int flywheelMotor1ID = 53;
        public static final Current statorCurrentLimit = Amps.of(40);
        public static final int flywheelMotor2ID = 54;
        public static final PerUnit<VoltageUnit, AngularVelocityUnit> VoltsPerRotationsPerSecond = Volts.per(RotationsPerSecond) ;
        public static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kS = VoltsPerRotationsPerSecond.of(0.1); // TODO
        public static final Measure<? extends PerUnit<VoltageUnit, AngularVelocityUnit>> kV = VoltsPerRotationsPerSecond.of(0.1); // TODO
        public static final Voltage maxVoltage = Volts.of(8);
        public static final Voltage bangBangVolts = Volts.of(5); // TODO
        public static final double gearRatio = 1d/1d;
        public static final AngularVelocity flywheelTolerance = RPM.of(50); // TODO
    }

    public static class HoodConstants {
        public static final int hoodLinearActuatorPWMID = 0;
        public static final Distance actuatorLength = Millimeters.of(100);
        public static final Distance hoodPivotToActuatorMount = Inches.of(10);
        public static final Distance actuatorMountToHoodEdge = Inches.of(6.5);
        public static final Angle minAngle = Degrees.of(0);//TODO
        public static final Angle maxAngle = Degrees.of(45); // TODO
        public static final Angle angleOffset = Degrees.of(0); // mechanical zero offset

    }
}

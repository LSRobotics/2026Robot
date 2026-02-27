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
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;
public class ShooterConstants {
    public static class FlywheelConstants {
        public static final AngularVelocity maxSpeed = RPM.of(100);// TODO
        public static final AngularVelocity holdSpeed = RPM.of(1); // TODO
        public static final int flywheelMotor1ID = 53;
        public static final Current statorCurrentLimit = Amps.of(80);//TODO
        public static final int flywheelMotor2ID = 54;
        public static final Voltage maxVoltage = Volts.of(11);
        public static final Voltage bangBangVolts = Volts.of(5); // TODO
        public static final double gearRatio = 1d/1d;
        public static final AngularVelocity flywheelTolerance = RPM.of(50); // TODO

        //From sysid
        public static final Per<VoltageUnit, AngularVelocityUnit> kVVoltSecondsPerRadian = VoltsPerRadianPerSecond.ofNative(0.018378); 
        public static final Voltage kS = Volts.of(0.089568);
        //derived from sysid (kVVoltSecondsPerRadian)
        public static final PerUnit<VoltageUnit, AngularVelocityUnit> VoltsPerRotationsPerSecond = Volts.per(RotationsPerSecond) ;
        public static final double kV = VoltsPerRotationsPerSecond.convertFrom(kVVoltSecondsPerRadian.in(VoltsPerRadianPerSecond), VoltsPerRadianPerSecond);
    }

    public static class HoodConstants {
        public static final int hoodLinearActuatorPWMID = 0;
        public static final int hoodLinearActuatorPWMID2 = 1;
        public static final Distance actuatorLengthExtended = Inches.of(9);
        public static final Distance actuatorLengthRetracted = Inches.of(6.75);

        public static final Distance hoodPivotToActuatorMount = Inches.of(5.5); //From retracted position
        public static final Distance hoodPivotToHoodEdge = Inches.of(5.5);
        public static final Angle maxAngle = Degrees.of(35);
        public static final Angle angleOffset = Degrees.of(11); // mechanical zero offset
        public static final Angle minAngle = angleOffset;//TODO


    }
}

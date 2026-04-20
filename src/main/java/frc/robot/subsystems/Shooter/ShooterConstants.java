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
        public static final AngularVelocity maxSpeed = RotationsPerSecond.of(100);// TODO
        public static final AngularVelocity holdSpeed = RotationsPerSecond.of(1); // TODO
        public static final int flywheelMotor1ID = 53;
        public static final Current statorCurrentLimit = Amps.of(80);//TODO
        public static final int flywheelMotor2ID = 54;
        public static final Voltage maxVoltage = Volts.of(12);
        public static final Voltage bangBangVolts = Volts.of(5.5); 
        public static final double gearRatio = 1d/1d;
        public static final AngularVelocity flywheelTolerance = RotationsPerSecond.of(8); // TODO

        public static final double bangBangCoefficient = 0.28;

        //From sysid
        public static final Per<VoltageUnit, AngularVelocityUnit> kVVoltSecondsPerRadian = VoltsPerRadianPerSecond.ofNative(0.018398); 
        public static final Per<VoltageUnit, AngularVelocityUnit> kAVoltSecondsPerRadian = VoltsPerRadianPerSecond.ofNative(0.001585); 

        public static final Voltage kS = Volts.of(0.10653);
        //derived from sysid (kVVoltSecondsPerRadian)
        public static final PerUnit<VoltageUnit, AngularVelocityUnit> VoltsPerRotationsPerSecond = Volts.per(RotationsPerSecond);
        public static final double kV = VoltsPerRotationsPerSecond.convertFrom(kVVoltSecondsPerRadian.in(VoltsPerRadianPerSecond), VoltsPerRadianPerSecond);
        public static final double kA = VoltsPerRotationsPerSecond.convertFrom(kAVoltSecondsPerRadian.in(VoltsPerRadianPerSecond), VoltsPerRadianPerSecond);

        public static final AngularVelocity manualSpeed1 = RotationsPerSecond.of(38);
        public static final AngularVelocity manualSpeed2 = RotationsPerSecond.of(40);
        public static final AngularVelocity manualSpeed3 = RotationsPerSecond.of(42);
        public static final AngularVelocity manualSpeed4 = RotationsPerSecond.of(45);

        public static final double rightTrenchSpeed = 45;
        public static final double leftTrenchSpeed = 42;
    }

    public static class HoodConstants {
        public static final int hoodLinearActuatorPWMID = 0;
        public static final int hoodLinearActuatorPWMID2 = 1;
        public static final Distance actuatorLengthExtended = Inches.of(9);
        public static final Distance actuatorLengthRetracted = Inches.of(6.75);

        public static final Distance hoodPivotToActuatorMount = Inches.of(9.5); //From retracted position
        public static final Distance hoodPivotToHoodEdge = Inches.of(5.3);
        public static final Angle maxAngle = Degrees.of(25);
        public static final Angle angleOffset = Degrees.of(0); // mechanical zero offset
        public static final Angle minAngle = Degrees.of(0);


    }

    public static AngularVelocity FeedSpeed = RotationsPerSecond.of(53);
    public static double FeedHood = 0.8;

    public static AngularVelocity FeedSpeed2 = RotationsPerSecond.of(89);
    public static double FeedHood2 = 0.8;
    

}

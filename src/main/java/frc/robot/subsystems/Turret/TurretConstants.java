package frc.robot.subsystems.Turret;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.AbstractExecutorService;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Time;
public class TurretConstants {
    public static final Translation2d turretOffset = new Translation2d(Inches.of(-9), Inches.of(-5)); //From center
    public static final double turretGearRatio = 0.1d;
    public static final Angle turretTolerance = Degrees.of(0.1);
    public static final AngularVelocity maxSpeed = DegreesPerSecond.of(50);
    public static final Pose2d hubBlue = new Pose2d(Meters.of(4.5), Meters.of(4.204), new Rotation2d());
    public static final Pose2d hubRed = new Pose2d(Meters.of(11.909), Meters.of(4.204), new Rotation2d());
    public static final int turretMotorID = 62;
    public static final double maxControlSpeed = 0.2;
    public static final double maxOpSpeed = 0.06;
    public static final Time lookaheadLatency = Seconds.of(1.2);
    public static final Angle turretRangeOneWay = Degrees.of(93);
    //new PIDController(0.008, 0, 0.0001);
    public static final double kP = 0.01277;
    public static final double kD = 0.00101;
    public static final double tolerance = 0.5;

    public static final Angle manualAngle1 = Degrees.of(0);
    public static final Angle manualAngle2 = Degrees.of(-90);
    public static final Angle manualAngle3 = Degrees.of(90);

    //Feed locations
    public static final Translation2d blue1 = new Translation2d(Meters.of(1.5), Meters.of(1)); //Outpost
    public static final Translation2d blue2 = new Translation2d(Meters.of(1.5), Meters.of(7)); //Depot
}

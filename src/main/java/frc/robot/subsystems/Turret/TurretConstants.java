package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.AbstractExecutorService;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
public class TurretConstants {
    public static final Translation2d turretOffset = new Translation2d(Inches.of(-9), Inches.of(-5)); //From center
    public static final double turretGearRatio = 0.1d;
    public static final Angle turretTolerance = Degrees.of(0.1);
    public static final AngularVelocity maxSpeed = DegreesPerSecond.of(50);
    public static final Pose2d hubBlue = new Pose2d(Meters.of(4.5), Meters.of(4.204), new Rotation2d());
    public static final Pose2d hubRed = new Pose2d(Meters.of(11.909), Meters.of(4.204), new Rotation2d());
    public static final int turretMotorID = 62;
    public static final double maxControlSpeed = 0.15;
    public static final double maxOpSpeed = 0.06;
    public static final Time lookaheadLatency = Seconds.of(1.2);
    public static final Angle turretRangeOneWay = Degrees.of(93);
    //new PIDController(0.008, 0, 0.0001);
    public static final double kP = 0.01876;
    public static final double kD = 0.00116;
    public static final double tolerance = 0.5;

    public static final Angle manualAngle1 = Degrees.of(0);
    public static final Angle manualAngle2 = Degrees.of(-90);
    public static final Angle manualAngle3 = Degrees.of(90);
}

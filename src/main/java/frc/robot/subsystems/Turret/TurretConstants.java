package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Second;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

public class TurretConstants {
    public static final Translation2d turretOffset = new Translation2d(0.0, 0.0); //From center
    public static final double turretGearRatio = 10d;
    public static final Angle turretTolerance = Degrees.of(0.5);
    public static final AngularVelocity maxSpeed = DegreesPerSecond.of(50);
    public static final Pose2d hubBlue = new Pose2d(Meters.of(4.5), Meters.of(4.204), new Rotation2d());
    public static final Pose2d hubRed = new Pose2d(Meters.of(11.909), Meters.of(4.204), new Rotation2d());
    public static final int turretMotorID = 62;
    public static final double maxControlSpeed = 0.5;
    public static final Time lookaheadLatency = Milliseconds.of(10);
    public static final Angle turretRangeOneWay = Degrees.of(90);
    //new PIDController(0.008, 0, 0.0001);
    public static final double kP = 0.006;
    public static final double kD = 0.0001;
    public static final double tolerance = 0.2;

}

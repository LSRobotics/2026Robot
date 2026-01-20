package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public class TurretConstants {
    public static final Translation2d turretOffset = new Translation2d(0.0, 0.0); //From center
    public static final double turretGearRatio = 1/1;

    public static final AngularVelocity maxSpeed = DegreesPerSecond.of(50);

    public static final Pose2d hubBlue = new Pose2d(Meters.of(4.5), Meters.of(4), new Rotation2d());

}


package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.Turret.TurretSubsystem;
import frc.robot.subsystems.leds.LEDConstants;
import frc.robot.subsystems.leds.LEDManager;
import frc.robot.subsystems.leds.LedsIOBlinkin;
import frc.robot.util.MathUtils;

public class ShootAtHubCommand extends Command {
    @SuppressWarnings("PMD.UnusedPrivateField")
    private final TurretSubsystem m_Turret;
    private final ShooterSubsystem m_Shooter;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedSupplier;
    private  double LEDColor = LEDConstants.defaultColor;

    private final Supplier<Double> leftRightTrim;
    private final Supplier<Double> latencyCompensation;


    // private PIDController turretPID = new PIDController(0.008, 0, 0.0001);
    private PIDController turretPID = new PIDController(TurretConstants.kP, 0, TurretConstants.kD);
    private BangBangController flywheelController = new BangBangController();
    // @SuppressWarnings("unchecked")
    private SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(
            ShooterConstants.FlywheelConstants.kS.in(Volts),
            ShooterConstants.FlywheelConstants.kV,
            ShooterConstants.FlywheelConstants.kA);
    private final Pose2d targetHubPose;
    

    public ShootAtHubCommand(TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem,
            Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedSupplier) {
        this.m_Turret = turretSubsystem;
        this.m_Shooter = shooterSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;
        this.chassisSpeedSupplier = chassisSpeedSupplier;
        this.leftRightTrim = () -> 0.0;
        this.latencyCompensation = () -> 0.0;
        this.targetHubPose = DriverStation.getAlliance().orElse(Alliance.Blue) == DriverStation.Alliance.Blue
                ? TurretConstants.hubBlue
                : TurretConstants.hubRed;
        addRequirements(m_Turret, m_Shooter);

        flywheelController.setTolerance(ShooterConstants.FlywheelConstants.flywheelTolerance.in(RotationsPerSecond));
        AimingConstants.initialize();
        turretPID.setTolerance(TurretConstants.turretTolerance.in(Degrees));
        Logger.recordOutput("Aiming/TargetHubPose", targetHubPose);
    }


    public ShootAtHubCommand(TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem,
            Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedSupplier, Supplier<Double> leftRightTrim, Supplier<Double> latencyCompensation) {
        this.m_Turret = turretSubsystem;
        this.m_Shooter = shooterSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;
        this.chassisSpeedSupplier = chassisSpeedSupplier;
        this.leftRightTrim = leftRightTrim;
        this.latencyCompensation = latencyCompensation;
        this.targetHubPose = DriverStation.getAlliance().orElse(Alliance.Blue) == DriverStation.Alliance.Blue
                ? TurretConstants.hubBlue
                : TurretConstants.hubRed;
        addRequirements(m_Turret, m_Shooter);

        flywheelController.setTolerance(ShooterConstants.FlywheelConstants.flywheelTolerance.in(RotationsPerSecond));
        AimingConstants.initialize();
        turretPID.setTolerance(TurretConstants.turretTolerance.in(Degrees));
        Logger.recordOutput("Aiming/TargetHubPose", targetHubPose);
    }

    @Override
    public void initialize() {
        LEDManager.setColor(LEDConstants.colorGold);
    }

    @Override
    public void execute() {
        Pose2d robotPose = robotPoseSupplier.get();
        ChassisSpeeds chassisSpeeds = chassisSpeedSupplier.get();

        Pose2d turretPose = robotPose.transformBy(
                new Transform2d(TurretConstants.turretOffset, new Rotation2d()));

        Translation2d robotVelocity = new Translation2d(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond).rotateBy(robotPose.getRotation());

        Translation2d turretOffset = TurretConstants.turretOffset;
        Translation2d tangentialVelocity = new Translation2d(
                -chassisSpeeds.omegaRadiansPerSecond * turretOffset.getY(),
                chassisSpeeds.omegaRadiansPerSecond * turretOffset.getX());

        Translation2d relVelocity = robotVelocity.plus(tangentialVelocity);

        Translation2d relPosition = targetHubPose.getTranslation()
                .minus(turretPose.getTranslation());
        double distanceToTarget = relPosition.getNorm();

        Logger.recordOutput("Aiming/DistanceToTarget", distanceToTarget);
        Logger.recordOutput("Aiming/TargetHubPose", targetHubPose);
        Logger.recordOutput("Aiming/TurretPose", turretPose);

        double targetRPM = AimingConstants.flywheelSpeedMap.get(distanceToTarget);
        double TOF = AimingConstants.flywheelTOFMap.get(distanceToTarget);
        double oldRPM = targetRPM;
        double predictedDistance = distanceToTarget;

        for (int i = 0; i < AimingConstants.maxIterations; i++) {
            Translation2d predictedTurretTranslation = turretPose.getTranslation().plus(relVelocity.times(TOF));

            Translation2d predictedRelPosition = targetHubPose.getTranslation().minus(predictedTurretTranslation);
            predictedDistance = predictedRelPosition.getNorm();

            targetRPM = AimingConstants.flywheelSpeedMap.get(predictedDistance);
            Logger.recordOutput("Aiming/RawSpeedRPM", targetRPM);

            double difference = targetRPM - oldRPM;
            if (Math.abs(difference) > AimingConstants.maxRPMChange * TOF) {
                targetRPM = oldRPM + Math.signum(difference) * AimingConstants.maxRPMChange * TOF;
            }

            double newTOF = AimingConstants.flywheelTOFMap.get(predictedDistance);

            Logger.recordOutput("Aiming/Iteration", i);
            Logger.recordOutput("Aiming/PredictedDistance", predictedDistance);
            Logger.recordOutput("Aiming/TargetRPM", targetRPM);
            Logger.recordOutput("Aiming/TOF", TOF);

            if (Math.abs(newTOF - TOF) < AimingConstants.ToFtolerance) {
                break;
            }

            TOF = newTOF;
            oldRPM = targetRPM;
        }

        TOF += latencyCompensation.get();

        Translation2d predictedTurretTranslation = turretPose.getTranslation().plus(relVelocity.times(TOF));

        LEDManager.setColor(LEDColor);

        double hoodAngleDeg = AimingConstants.hoodAngleMap.get(predictedDistance);
        setHoodAngle(hoodAngleDeg);
        aimTurret(predictedTurretTranslation, robotPose.getRotation(), targetHubPose);
        spinUpFlywheel(targetRPM);
    }

    public void spinUpFlywheel(double targetRPM) {

        double targetRPS = targetRPM / 60.0;

        double feedforwardVoltage = flywheelFeedforward.calculate(targetRPS); // Tuned in V per rot/s
        double controlOutput = flywheelController.calculate(
                m_Shooter.getFlywheelVelocity().times(ShooterConstants.FlywheelConstants.gearRatio)
                        .in(RotationsPerSecond),
                targetRPS)
                * ShooterConstants.FlywheelConstants.bangBangVolts.in(Volt);
        double totalVoltage = feedforwardVoltage
                + controlOutput * ShooterConstants.FlywheelConstants.bangBangCoefficient;
        totalVoltage = MathUtil.clamp(totalVoltage, -ShooterConstants.FlywheelConstants.maxVoltage.in(Volt),
                ShooterConstants.FlywheelConstants.maxVoltage.in(Volt));

        Logger.recordOutput("Aiming/Flywheel/TargetRPM", targetRPM);
        Logger.recordOutput("Aiming/Flywheel/FeedforwardVoltage", feedforwardVoltage);
        Logger.recordOutput("Aiming/Flywheel/TotalVoltage", totalVoltage);

        m_Shooter.targetSpeed = RPM.of(targetRPM);

        m_Shooter.setFlywheelVoltage(Volt.of(totalVoltage));
    }

    public void aimTurret(Translation2d predictedTurretPosition, Rotation2d robotRotation, Pose2d target) {
        Translation2d toTarget = target.getTranslation().minus(predictedTurretPosition);

        Rotation2d fieldAngle = toTarget.getAngle();

        Rotation2d turretAngle = fieldAngle.minus(robotRotation).unaryMinus();

        turretAngle = turretAngle.plus(Rotation2d.fromDegrees(leftRightTrim.get()));

        double setpoint = MathUtil.clamp(
                turretAngle.getDegrees(),
                -TurretConstants.turretRangeOneWay.in(Degrees),
                TurretConstants.turretRangeOneWay.in(Degrees));

        turretPID.setSetpoint(setpoint);

        double speed = turretPID.calculate(m_Turret.getAngle().in(Degrees));
        Logger.recordOutput("Aiming/Turret/Unclamped Speed", speed);

        speed = MathUtils.clamp(-TurretConstants.maxControlSpeed, TurretConstants.maxControlSpeed, speed);

        m_Turret.setSpeed(speed);

        Logger.recordOutput("Aiming/Turret/PID_Error", turretPID.getError());
        Logger.recordOutput("Aiming/Turret/Speed", speed);
        Logger.recordOutput("Aiming/Turret/PID_Setpoint", turretPID.getSetpoint());

        if (Math.abs(m_Turret.getAngle().in(Degrees)-turretAngle.getDegrees())<5){
            LEDColor = LEDConstants.colorLimeGreen;
        }
        else {
            LEDColor = LEDConstants.colorRed;
        }

    }

    public void setHoodAngle(double a) {
        m_Shooter.setHoodPosition(a);
        Logger.recordOutput("Aiming/Hood/Angle", a);
    }

    @Override
    public void end(boolean interrupted) {
        m_Turret.setSpeed(0);
        m_Shooter.setFlywheelVoltage(Volt.of(0));
        m_Shooter.setHoodPosition(-1d);
        LEDManager.setDefault();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    /*
     * TODO: FInalize
     * 1.Start with mid hood angle
     * 
     * 2.Adjust hood until trajectory is good (Prefer higher arcs and lower rpm )
     * 
     * 3.Adjust RPM until consistent
     * 
     * 4. test 5 balls
     * 
     * 5. Measure average TOF (From leaving flywheel to passing top plane of hub
     * funnel)
     * 
     * 6.Populate all 3 tables together
     */
    // First: 2m, 3m, 4m, 5m
    // Second: 0.5m, 1m, 1.5m, 2.5m, 3.5m, 4.5m, 5.5m, 6m
    // Third: 2.25m, 2.75m, 3.25m, 3.75m, 4.25m

    private class AimingConstants {
        public static final int maxIterations = 4;
        public static final double ToFtolerance = 0.06; // seconds
        public static final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap(); // Meters to
                                                                                                            // RPM at
                                                                                                            // best hood
                                                                                                            // angle
        public static final InterpolatingDoubleTreeMap flywheelTOFMap = new InterpolatingDoubleTreeMap(); // Meters
                                                                                                          // toseconds
                                                                                                          // in air at
                                                                                                          // best hood
                                                                                                          // angle
        public static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap(); // Meters to
                                                                                                        // hood position
                                                                                                        // as percent
                                                                                                        // from center
        public static final double maxRPMChange = 4500; // RPM per second TODO: tune this

        public static void initialize() { // TODO: fill out once bot is done
            // Meters to RPM
            flywheelSpeedMap.put(Meters.convertFrom(261, Inches), RPM.convertFrom(57, RotationsPerSecond));
            // Meters to tof
            flywheelTOFMap.put(Meters.convertFrom(261, Inches), Seconds.of(1.235).in(Seconds));
            // Meters to hood angle at hoodTestRPM
            hoodAngleMap.put(Meters.convertFrom(261, Inches), 0.8d);

            flywheelSpeedMap.put(Meters.convertFrom(239, Inches), RPM.convertFrom(52, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(239, Inches), Seconds.of(1.23).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(239, Inches), 0.6d);

            flywheelSpeedMap.put(Meters.convertFrom(210, Inches), RPM.convertFrom(50, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(210, Inches), Seconds.of(1.23).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(210, Inches), 0.4d);

            flywheelSpeedMap.put(Meters.convertFrom(192, Inches), RPM.convertFrom(49, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(192, Inches), Seconds.of(1.25).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(192, Inches), 0.3d);

            flywheelSpeedMap.put(Meters.convertFrom(173, Inches), RPM.convertFrom(47, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(173, Inches), Seconds.of(1.235).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(173, Inches), 0.1d);

            flywheelSpeedMap.put(Meters.convertFrom(139, Inches), RPM.convertFrom(44, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(139, Inches), Seconds.of(1.23).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(139, Inches), -0.18d);

            flywheelSpeedMap.put(Meters.convertFrom(115, Inches), RPM.convertFrom(43, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(115, Inches), Seconds.of(1.24).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(115, Inches), -0.4d);

            flywheelSpeedMap.put(Meters.convertFrom(99, Inches), RPM.convertFrom(42, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(99, Inches), Seconds.of(1.22).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(99, Inches), -0.8d);

            flywheelSpeedMap.put(Meters.convertFrom(89, Inches), RPM.convertFrom(41, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(89, Inches), Seconds.of(1.23).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(89, Inches), -1d);

            flywheelSpeedMap.put(Meters.convertFrom(82, Inches), RPM.convertFrom(37, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(82, Inches), Seconds.of(1.04).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(82, Inches), -1d);
            
            flywheelSpeedMap.put(Meters.convertFrom(65, Inches), RPM.convertFrom(36, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(65, Inches), Seconds.of(1.02).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(65, Inches), -1d);

            flywheelSpeedMap.put(Meters.convertFrom(0, Inches), RPM.convertFrom(36, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(0, Inches), Seconds.of(1.02).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(0, Inches), -1d);
        }

        public AimingConstants() throws Exception {
            throw new Exception("dont instantiate this");
        }
    }

}

/*
 * // Meters to RPM
 * flywheelSpeedMap.put(Meters.convertFrom(44, Inches), RPM.convertFrom(40,
 * RotationsPerSecond));
 * 
 * flywheelSpeedMap.put(Meters.convertFrom(48, Inches), RPM.convertFrom(35,
 * RotationsPerSecond));
 * 
 * flywheelSpeedMap.put(Meters.convertFrom(78, Inches), RPM.convertFrom(45,
 * RotationsPerSecond));
 * 
 * flywheelSpeedMap.put(Meters.convertFrom(105, Inches), RPM.convertFrom(44,
 * RotationsPerSecond));
 * 
 * flywheelSpeedMap.put(Meters.convertFrom(132, Inches), RPM.convertFrom(50,
 * RotationsPerSecond));
 * 
 * flywheelSpeedMap.put(Meters.convertFrom(139, Inches), RPM.convertFrom(52,
 * RotationsPerSecond));
 * 
 * flywheelSpeedMap.put(Meters.convertFrom(153, Inches), RPM.convertFrom(55,
 * RotationsPerSecond));
 * 
 * flywheelSpeedMap.put(Meters.convertFrom(168, Inches), RPM.convertFrom(95,
 * RotationsPerSecond));
 * 
 * flywheelSpeedMap.put(Meters.convertFrom(208, Inches), RPM.convertFrom(52,
 * RotationsPerSecond));
 * 
 * flywheelSpeedMap.put(Meters.convertFrom(236, Inches), RPM.convertFrom(54,
 * RotationsPerSecond));
 * 
 * 
 * 
 * 
 * 
 * // Meters to tof
 * flywheelTOFMap.put(Meters.convertFrom(44, Inches),
 * Seconds.of(1.14).in(Seconds));
 * 
 * flywheelTOFMap.put(Meters.convertFrom(48, Inches),
 * Seconds.of(0.89).in(Seconds));
 * 
 * flywheelTOFMap.put(Meters.convertFrom(78, Inches),
 * Seconds.of(1.34).in(Seconds));
 * 
 * flywheelTOFMap.put(Meters.convertFrom(105, Inches),
 * Seconds.of(1.28).in(Seconds));
 * 
 * flywheelTOFMap.put(Meters.convertFrom(132, Inches),
 * Seconds.of(1.52).in(Seconds));
 * 
 * flywheelTOFMap.put(Meters.convertFrom(139, Inches),
 * Seconds.of(1.56).in(Seconds));
 * 
 * flywheelTOFMap.put(Meters.convertFrom(153, Inches),
 * Seconds.of(1.55).in(Seconds));
 * 
 * flywheelTOFMap.put(Meters.convertFrom(168, Inches),
 * Seconds.of(1.87).in(Seconds));
 * 
 * flywheelTOFMap.put(Meters.convertFrom(208, Inches),
 * Seconds.of(1.38).in(Seconds));
 * 
 * flywheelTOFMap.put(Meters.convertFrom(236, Inches),
 * Seconds.of(1.15).in(Seconds));
 * 
 * 
 * // Meters to hood angle at hoodTestRPM
 * hoodAngleMap.put(Meters.convertFrom(44, Inches), -1d);
 * 
 * hoodAngleMap.put(Meters.convertFrom(48, Inches), -1d);
 * 
 * hoodAngleMap.put(Meters.convertFrom(78, Inches), -1d);
 * 
 * hoodAngleMap.put(Meters.convertFrom(105, Inches), -1d);
 * 
 * hoodAngleMap.put(Meters.convertFrom(139, Inches), -1d);
 * 
 * hoodAngleMap.put(Meters.convertFrom(139, Inches), -1d);
 * 
 * hoodAngleMap.put(Meters.convertFrom(153, Inches), -1d);
 * 
 * hoodAngleMap.put(Meters.convertFrom(168, Inches), -1d);
 * 
 * hoodAngleMap.put(Meters.convertFrom(208, Inches), -0.45090180360721444);
 * 
 * hoodAngleMap.put(Meters.convertFrom(236, Inches), 0.20040080160320642);
 * 
 */
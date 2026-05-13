package frc.robot.commands;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volt;
import static org.wpilib.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.controller.BangBangController;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command3.Command;

import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterMechanism;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.Turret.TurretMechanism;
import frc.robot.subsystems.leds.LEDConstants;
import frc.robot.subsystems.leds.LEDManager;
import frc.robot.util.FieldConstants;
import frc.robot.util.MathUtils;

public class ShootingCommands {

    // Returns the new LED color based on turret error, so the main loop can use it
    private static double aimTurret(
            TurretMechanism turret,
            PIDController turretPID,
            Translation2d predictedTurretPosition,
            Rotation2d robotRotation,
            Pose2d target,
            Supplier<Double> leftRightTrim) {

        Translation2d toTarget = target.getTranslation().minus(predictedTurretPosition);
        Rotation2d fieldAngle = toTarget.getAngle();
        Rotation2d turretAngle = fieldAngle.minus(robotRotation).unaryMinus();
        turretAngle = turretAngle.plus(Rotation2d.fromDegrees(leftRightTrim.get()));

        double setpoint = MathUtils.clamp(
                -TurretConstants.turretRangeOneWay.in(Degrees),
                TurretConstants.turretRangeOneWay.in(Degrees), 
                turretAngle.getDegrees());

        turretPID.setSetpoint(setpoint);

        double speed = turretPID.calculate(turret.getAngle().in(Degrees));
        Logger.recordOutput("Aiming/Turret/Unclamped Speed", speed);
        speed = MathUtils.clamp(-TurretConstants.maxControlSpeed, TurretConstants.maxControlSpeed, speed);
        turret.setSpeed(speed);

        Logger.recordOutput("Aiming/Turret/PID_Error", turretPID.getError());
        Logger.recordOutput("Aiming/Turret/Speed", speed);
        Logger.recordOutput("Aiming/Turret/PID_Setpoint", turretPID.getSetpoint());

        return Math.abs(turret.getAngle().in(Degrees) - turretAngle.getDegrees()) < 5
                ? LEDConstants.colorLimeGreen
                : LEDConstants.colorRed;
    }

    private static void spinUpFlywheel(
            ShooterMechanism shooter,
            BangBangController flywheelController,
            SimpleMotorFeedforward flywheelFeedforward,
            double targetRPM) {

        double targetRPS = targetRPM / 60.0;

        double feedforwardVoltage = flywheelFeedforward.calculate(targetRPS);
        double controlOutput = flywheelController.calculate(
                shooter.getFlywheelVelocity()
                        .times(ShooterConstants.FlywheelConstants.gearRatio)
                        .in(RotationsPerSecond),
                targetRPS)
                * ShooterConstants.FlywheelConstants.bangBangVolts.in(Volt);

        double totalVoltage = MathUtils.clamp(-ShooterConstants.FlywheelConstants.maxVoltage.in(Volt), ShooterConstants.FlywheelConstants.maxVoltage.in(Volt), feedforwardVoltage + controlOutput * ShooterConstants.FlywheelConstants.bangBangCoefficient);

        Logger.recordOutput("Aiming/Flywheel/TargetRPM", targetRPM);
        Logger.recordOutput("Aiming/Flywheel/FeedforwardVoltage", feedforwardVoltage);
        Logger.recordOutput("Aiming/Flywheel/TotalVoltage", totalVoltage);

        shooter.targetSpeed = RPM.of(targetRPM);
        shooter.setFlywheelVoltage(Volt.of(totalVoltage));
    }

    private static void setHoodAngle(ShooterMechanism shooter, double angle) {
        shooter.setHoodPosition(angle);
        Logger.recordOutput("Aiming/Hood/Angle", angle);
    }

    public static Command feed(
        TurretMechanism turret,
        ShooterMechanism shooter,
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<ChassisVelocities> chassisSpeedSupplier) {

        return Command.requiring(turret, shooter).executing(co -> {
            PIDController pid = new PIDController(TurretConstants.kP, 0, TurretConstants.kD);
            BangBangController flywheelController = new BangBangController();
            SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(
                    ShooterConstants.FlywheelConstants.kS.in(Volts),
                    ShooterConstants.FlywheelConstants.kV,
                    ShooterConstants.FlywheelConstants.kA);

            pid.setTolerance(TurretConstants.turretTolerance.in(Degrees));
            flywheelController.setTolerance(
                    ShooterConstants.FlywheelConstants.flywheelTolerance.in(RotationsPerSecond));

            while (true) {
                Pose2d target1 = new Pose2d(TurretConstants.blue1, new Rotation2d());
                Pose2d target2 = new Pose2d(TurretConstants.blue2, new Rotation2d());

                if (DriverStation.getAlliance().orElse(Alliance.BLUE) == Alliance.RED) {
                    target1 = new Pose2d(
                            FieldConstants.fieldLength - target1.getX(),
                            target1.getY(),
                            target1.getRotation().plus(Rotation2d.fromDegrees(180)));
                    target2 = new Pose2d(
                            FieldConstants.fieldLength - target2.getX(),
                            target2.getY(),
                            target2.getRotation().plus(Rotation2d.fromDegrees(180)));
                }

                Pose2d robotPose = robotPoseSupplier.get();
                Pose2d target = target1.getTranslation().getDistance(robotPose.getTranslation())
                        < target2.getTranslation().getDistance(robotPose.getTranslation())
                        ? target1 : target2;

                Pose2d turretPose = robotPose.transformBy(
                        new Transform2d(TurretConstants.turretOffset, new Rotation2d()));

                ChassisVelocities chassisSpeeds = chassisSpeedSupplier.get();
                Pose2d predictedTurretPose = turretPose.plus(new Transform2d(
                        new Translation2d(
                                chassisSpeeds.vx * TurretConstants.lookaheadLatency.in(Seconds),
                                chassisSpeeds.vy * TurretConstants.lookaheadLatency.in(Seconds)),
                        new Rotation2d(
                                chassisSpeeds.omega * TurretConstants.lookaheadLatency.in(Seconds))));

                Rotation2d angleToTarget = target.getTranslation()
                        .minus(predictedTurretPose.getTranslation())
                        .getAngle()
                        .minus(robotPose.getRotation())
                        .unaryMinus();

                pid.setSetpoint(MathUtils.clamp(
                        -TurretConstants.turretRangeOneWay.in(Degrees),
                        TurretConstants.turretRangeOneWay.in(Degrees),
                        angleToTarget.getDegrees()));

                double speed = MathUtils.clamp(
                        -TurretConstants.maxControlSpeed,
                        TurretConstants.maxControlSpeed,
                        pid.calculate(turret.getAngle().in(Degrees)));

                turret.setSpeed(speed);

                Logger.recordOutput("Turret/PID_Error", pid.getError());
                Logger.recordOutput("Turret/PID_Setpoint", pid.getSetpoint());

                double LEDColor = Math.abs(turret.getAngle().in(Degrees) - angleToTarget.getDegrees()) < 5
                        ? LEDConstants.colorLimeGreen
                        : LEDConstants.colorRed;

                spinUpFlywheel(shooter, flywheelController, flywheelFeedforward,
                        ShooterConstants.FeedSpeed.in(RPM));
                setHoodAngle(shooter, ShooterConstants.FeedHood);
                LEDManager.setColor(LEDColor);

                co.yield();
            }
            }).whenCanceled(() -> {
                turret.setSpeed(0);
                shooter.setFlywheelVoltage(Volt.of(0));
                shooter.setHoodPosition(-1d);
                LEDManager.setDefault();
            }).named("Feed");
    }

    public static Command feed2(
        TurretMechanism turret,
        ShooterMechanism shooter,
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<ChassisVelocities> chassisSpeedSupplier) {

        return Command.requiring(turret, shooter).executing(co -> {
            PIDController pid = new PIDController(TurretConstants.kP, 0, TurretConstants.kD);
            BangBangController flywheelController = new BangBangController();
            SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(
                    ShooterConstants.FlywheelConstants.kS.in(Volts),
                    ShooterConstants.FlywheelConstants.kV,
                    ShooterConstants.FlywheelConstants.kA);

            pid.setTolerance(TurretConstants.turretTolerance.in(Degrees));
            flywheelController.setTolerance(
                    ShooterConstants.FlywheelConstants.flywheelTolerance.in(RotationsPerSecond));

            while (true) {
                Pose2d target1 = new Pose2d(TurretConstants.blue1, new Rotation2d());
                Pose2d target2 = new Pose2d(TurretConstants.blue2, new Rotation2d());

                if (DriverStation.getAlliance().orElse(Alliance.BLUE) == Alliance.RED) {
                    target1 = new Pose2d(
                            FieldConstants.fieldLength - target1.getX(),
                            target1.getY(),
                            target1.getRotation().plus(Rotation2d.fromDegrees(180)));
                    target2 = new Pose2d(
                            FieldConstants.fieldLength - target2.getX(),
                            target2.getY(),
                            target2.getRotation().plus(Rotation2d.fromDegrees(180)));
                }

                Pose2d robotPose = robotPoseSupplier.get();
                Pose2d target = target1.getTranslation().getDistance(robotPose.getTranslation())
                        < target2.getTranslation().getDistance(robotPose.getTranslation())
                        ? target1 : target2;

                Pose2d turretPose = robotPose.transformBy(
                        new Transform2d(TurretConstants.turretOffset, new Rotation2d()));

                ChassisVelocities chassisSpeeds = chassisSpeedSupplier.get();
                Pose2d predictedTurretPose = turretPose.plus(new Transform2d(
                        new Translation2d(
                                chassisSpeeds.vx * TurretConstants.lookaheadLatency.in(Seconds),
                                chassisSpeeds.vy * TurretConstants.lookaheadLatency.in(Seconds)),
                        new Rotation2d(
                                chassisSpeeds.omega * TurretConstants.lookaheadLatency.in(Seconds))));

                Rotation2d angleToTarget = target.getTranslation()
                        .minus(predictedTurretPose.getTranslation())
                        .getAngle()
                        .minus(robotPose.getRotation())
                        .unaryMinus();

                pid.setSetpoint(MathUtils.clamp(
                        -TurretConstants.turretRangeOneWay.in(Degrees),
                        TurretConstants.turretRangeOneWay.in(Degrees),
                        angleToTarget.getDegrees()));

                double speed = MathUtils.clamp(
                        -TurretConstants.maxControlSpeed,
                        TurretConstants.maxControlSpeed,
                        pid.calculate(turret.getAngle().in(Degrees)));

                turret.setSpeed(speed);

                Logger.recordOutput("Turret/PID_Error", pid.getError());
                Logger.recordOutput("Turret/PID_Setpoint", pid.getSetpoint());

                double LEDColor = Math.abs(turret.getAngle().in(Degrees) - angleToTarget.getDegrees()) < 5
                        ? LEDConstants.colorLimeGreen
                        : LEDConstants.colorRed;

                spinUpFlywheel(shooter, flywheelController, flywheelFeedforward, ShooterConstants.FeedSpeed2.in(RPM));
                setHoodAngle(shooter, ShooterConstants.FeedHood2);
                LEDManager.setColor(LEDColor);

                co.yield();
            }
            }).whenCanceled(() -> {
                turret.setSpeed(0);
                shooter.setFlywheelVoltage(Volt.of(0));
                shooter.setHoodPosition(-1d);
                LEDManager.setDefault();
            }).named("Feed");
    }


    public static Command shootAtHub(
            TurretMechanism turret,
            ShooterMechanism shooter,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisVelocities> chassisSpeedSupplier,
            Supplier<Double> leftRightTrim,
            Supplier<Double> latencyCompensation) {

        return Command.requiring(turret, shooter)
            .executing(co -> {
                PIDController turretPID = new PIDController(TurretConstants.kP, 0, TurretConstants.kD);
                BangBangController flywheelController = new BangBangController();
                SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(
                        ShooterConstants.FlywheelConstants.kS.in(Volts),
                        ShooterConstants.FlywheelConstants.kV,
                        ShooterConstants.FlywheelConstants.kA);

                turretPID.setTolerance(TurretConstants.turretTolerance.in(Degrees));
                flywheelController.setTolerance(
                        ShooterConstants.FlywheelConstants.flywheelTolerance.in(RotationsPerSecond));
                AimingConstants.initialize();

                double LEDColor = LEDConstants.defaultColor;
                LEDManager.setColor(LEDConstants.colorGold);

                while (true) {
                    Pose2d targetHubPose = DriverStation.getAlliance().orElse(Alliance.BLUE) == Alliance.BLUE
                            ? TurretConstants.hubBlue
                            : TurretConstants.hubRed;

                    Pose2d robotPose = robotPoseSupplier.get();
                    ChassisVelocities chassisSpeeds = chassisSpeedSupplier.get();

                    Pose2d turretPose = robotPose.transformBy(
                            new Transform2d(TurretConstants.turretOffset, new Rotation2d()));

                    Translation2d robotVelocity = new Translation2d(
                            chassisSpeeds.vx,
                            chassisSpeeds.vy).rotateBy(robotPose.getRotation());

                    Translation2d turretOffset = TurretConstants.turretOffset;
                    Translation2d tangentialVelocity = new Translation2d(
                            -chassisSpeeds.omega * turretOffset.getY(),
                            chassisSpeeds.omega * turretOffset.getX());

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
                        Translation2d predictedTurretTranslation =
                                turretPose.getTranslation().plus(relVelocity.times(TOF));
                        Translation2d predictedRelPosition =
                                targetHubPose.getTranslation().minus(predictedTurretTranslation);
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

                    Translation2d predictedTurretTranslation =
                            turretPose.getTranslation().plus(relVelocity.times(TOF));

                    setHoodAngle(shooter, AimingConstants.hoodAngleMap.get(predictedDistance));
                    LEDColor = aimTurret(turret, turretPID, predictedTurretTranslation,
                            robotPose.getRotation(), targetHubPose, leftRightTrim);
                    spinUpFlywheel(shooter, flywheelController, flywheelFeedforward, 0.97 * targetRPM);

                    LEDManager.setColor(LEDColor);
                    co.yield();
                }
            })
            .whenCanceled(() -> {
                turret.setSpeed(0);
                shooter.setFlywheelVoltage(Volt.of(0));
                shooter.setHoodPosition(-1d);
                LEDManager.setDefault();
            })
            .named("Shoot At Hub");
    }

    public static Command shootAtHub(
            TurretMechanism turret,
            ShooterMechanism shooter,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisVelocities> chassisSpeedSupplier) {
        return shootAtHub(turret, shooter, robotPoseSupplier, chassisSpeedSupplier, () -> 0.0, () -> 0.0);
    }

    public static class AimingConstants {
        public static final int maxIterations = 4;
        public static final double ToFtolerance = 0.06;
        public static final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap flywheelTOFMap = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
        public static final double maxRPMChange = 4500;

        public static void initialize() {
            flywheelSpeedMap.put(Meters.convertFrom(270, Inches), RPM.convertFrom(55, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(270, Inches), Seconds.of(1.025).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(270, Inches), 0.55d);

            flywheelSpeedMap.put(Meters.convertFrom(249, Inches), RPM.convertFrom(53, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(249, Inches), Seconds.of(1.02).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(249, Inches), 0.5d);

            flywheelSpeedMap.put(Meters.convertFrom(233, Inches), RPM.convertFrom(50, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(233, Inches), Seconds.of(1.02).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(233, Inches), 0.3d);

            flywheelSpeedMap.put(Meters.convertFrom(198, Inches), RPM.convertFrom(45, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(198, Inches), Seconds.of(1.013).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(198, Inches), 0.25d);

            flywheelSpeedMap.put(Meters.convertFrom(187, Inches), RPM.convertFrom(44, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(187, Inches), Seconds.of(1.01).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(187, Inches), 0.15d);

            flywheelSpeedMap.put(Meters.convertFrom(164, Inches), RPM.convertFrom(42, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(164, Inches), Seconds.of(1.01).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(164, Inches), -0.05d);

            flywheelSpeedMap.put(Meters.convertFrom(142, Inches), RPM.convertFrom(40, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(142, Inches), Seconds.of(1.01).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(142, Inches), -0.2d);

            flywheelSpeedMap.put(Meters.convertFrom(118, Inches), RPM.convertFrom(38, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(118, Inches), Seconds.of(1.03).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(118, Inches), -0.4d);

            flywheelSpeedMap.put(Meters.convertFrom(98, Inches), RPM.convertFrom(37, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(98, Inches), Seconds.of(1.01).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(98, Inches), -0.65d);

            flywheelSpeedMap.put(Meters.convertFrom(82, Inches), RPM.convertFrom(36, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(82, Inches), Seconds.of(1.023).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(82, Inches), -0.95d);

            flywheelSpeedMap.put(Meters.convertFrom(67.67, Inches), RPM.convertFrom(34, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(67.67, Inches), Seconds.of(1.01).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(67.67, Inches), -1.0d);

            flywheelSpeedMap.put(Meters.convertFrom(0, Inches), RPM.convertFrom(34, RotationsPerSecond));
            flywheelTOFMap.put(Meters.convertFrom(0, Inches), Seconds.of(1.01).in(Seconds));
            hoodAngleMap.put(Meters.convertFrom(0, Inches), -1.0d);
        }

        private AimingConstants() {}
    }
}